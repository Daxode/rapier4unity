use std::ffi::{c_char, CString};
use std::os::raw::c_ulonglong;
use log::{Level, Metadata, Record};

#[repr(C,packed)]
#[allow(non_snake_case)]
struct IUnityInterfaces
{
    pub GetInterface: Option<unsafe extern "system" fn(guid: *const UnityInterfaceGUID) -> *mut IUnityInterface>,
    pub RegisterInterface: Option<unsafe extern "system" fn(guid: UnityInterfaceGUID, ptr: *mut IUnityInterface)>,
    pub GetInterfaceSplit: Option<unsafe extern "system" fn(guidHigh: c_ulonglong, guidLow: c_ulonglong) -> *mut IUnityInterface>,
    pub RegisterInterfaceSplit: Option<unsafe extern "system" fn(guidHigh: c_ulonglong, guidLow:c_ulonglong, ptr: *mut IUnityInterface)>,
}

#[repr(C,packed)]
struct IUnityInterface {
    pub add: u8,
}

#[repr(C,packed)]
#[derive(Default, Copy, Clone)]
pub struct UnityInterfaceGUID {
    pub m_guidhigh: ::std::os::raw::c_ulonglong,
    pub m_guidlow: ::std::os::raw::c_ulonglong,
}

// IUnityLog (0x9E7507fA5B444D5D, 0x92FB979515EA83FC)
const IUNITY_LOG_GUID: UnityInterfaceGUID =  UnityInterfaceGUID{m_guidhigh:0x9E7507fA5B444D5D_u64, m_guidlow:0x92FB979515EA83FC_u64};
#[repr(C,packed)]
#[allow(non_snake_case)]
struct IUnityLog
{
    pub Log: extern "system" fn(ttype: UnityLogType, message: *const c_char, fileName: *const c_char, fileLine: i32),
}

#[repr(u32)]
enum UnityLogType
{
    Error = 0,
    Warning = 2,
    Log = 3,
    // Exception = 4,
}

struct UnityLogger;

impl log::Log for UnityLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let log_type = match record.metadata().level() {
                Level::Error => UnityLogType::Error,
                Level::Warn => UnityLogType::Warning,
                Level::Info => UnityLogType::Log,
                Level::Debug => UnityLogType::Log,
                Level::Trace => UnityLogType::Log,
            };
            
            let message = CString::new(record.args().to_string()).unwrap();
            let file = file!();
            let line = line!();
            unsafe {
                let log = (*UNITY_LOG_PTR).Log;
                let val = CString::new(file).unwrap();
                log(log_type, message.as_ptr(), val.as_ptr(), line as i32);
            }
        }
    }

    fn flush(&self) {}
}

static mut UNITY_LOG_PTR: *const IUnityLog = std::ptr::null();

#[unsafe(no_mangle)]
#[allow(non_snake_case)]
extern "system" fn UnityPluginLoad(unityInterfacesPtr: *mut IUnityInterfaces)
{
    unsafe {
        let getInterface = (*unityInterfacesPtr).GetInterface;
        UNITY_LOG_PTR = getInterface.expect("Couldn't get unity interface log")(&IUNITY_LOG_GUID) as *const IUnityLog;
        let _ = log::set_logger(&UnityLogger).map(|()| log::set_max_level(Level::Info.to_level_filter()));
        log::trace!("Unity logger loaded");
    }
}

#[unsafe(no_mangle)]
extern "system" fn UnityPluginUnload() {
    log::trace!("Unity logger unloaded");
    unsafe {
        UNITY_LOG_PTR = std::ptr::null();
    }
}