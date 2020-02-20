use crate::messages::{Pose, Point, PointCloud};
use crate::robot::Command;

use libloading::{Library,Symbol};
use libloading::os::unix::Symbol as RawSymbol;
use std::ffi::{OsStr, CString};

pub trait ControllerPlugin {
    fn on_new_target(&self, target: Point);
    fn on_new_pose(&self, pose: Pose);
    fn on_new_scan(&self, cloud: PointCloud);
    fn get_command(&self) -> Command;
}

#[repr(C)]
pub struct Obj {
    value: [u8; 0]
}
pub struct ObjPtr(*mut Obj);

unsafe impl Send for ObjPtr {}
unsafe impl Sync for ObjPtr {}


type Init = extern "C" fn() -> *mut Obj;
type FreeData = extern "C" fn(*mut Obj);
type OnNewPose = extern "C" fn(*mut Obj, Pose);
type OnNewTarget = extern "C" fn(*mut Obj, Point);
type OnNewScan = extern "C" fn(*mut Obj, *const Point, usize);
type GetCommand = extern "C" fn(*mut Obj) -> Command;

struct VTable {
    init: RawSymbol<Init>,
    free_data: RawSymbol<FreeData>,
    on_new_target: RawSymbol<OnNewTarget>,
    on_new_pose: RawSymbol<OnNewPose>,
    on_new_scan: RawSymbol<OnNewScan>,
    get_command: RawSymbol<GetCommand>
}

unsafe fn load<T>(lib: &Library, name: &str) -> RawSymbol<T> {
    let cstring = CString::new(name).unwrap();
    let sym: Symbol<T> = lib.get(cstring.as_bytes())
        .expect(format!("{} is not defined", name).as_str());
    sym.into_raw()
}

impl VTable {
    unsafe fn new(library: &Library) -> Self {
        VTable {
            init: load(library, "init"),
            free_data: load(library, "free_data"),
            on_new_target: load(library, "on_new_target"),
            on_new_pose: load(library, "on_new_pose"),
            on_new_scan: load(library, "on_new_scan"),
            get_command: load(library, "get_command")
        }
    }
}

pub struct Plugin {
    #[allow(dead_code)]
    // this field is just to ensure that the library outlives the raw symbols in the table
    library: Library,
    object: ObjPtr,
    vtable: VTable
}


impl Plugin {
    pub unsafe fn new(library_name: &OsStr) -> Plugin {
        let library = Library::new(library_name).unwrap();
        let vtable = VTable::new(&library);

        let object: *mut Obj = (vtable.init)();

        Plugin {
            library: library,
            object: ObjPtr(object),
            vtable: vtable,
        }
    }
}

impl Drop for Plugin {
    fn drop(&mut self) {
        (self.vtable.free_data)(self.object.0);
    }
}

impl ControllerPlugin for Plugin {
    fn on_new_target(&self, target: Point) {
        (self.vtable.on_new_target)(self.object.0, target);
    }

    fn on_new_pose(&self, pose: Pose) {
        (self.vtable.on_new_pose)(self.object.0, pose)
    }

    fn on_new_scan(&self, cloud: PointCloud) {
        (self.vtable.on_new_scan)(self.object.0, cloud.points.as_ptr(), cloud.points.len())
    }

    fn get_command(&self) -> Command {
        (self.vtable.get_command)(self.object.0)
    }
}