


pub trait RecoverExt<T,E> {

    fn recover<F: Fn(E) -> T>(self, f: F) -> T;

}

impl<T,E> RecoverExt<T,E> for Result<T,E> {
    fn recover<F: Fn(E) -> T>(self, f: F) -> T {
        match self {
            Ok(t) => t,
            Err(e) => f(e)
        }
    }
}

pub trait LogExt<E> {
    fn log(self) -> ();
}

impl<E: std::fmt::Debug> LogExt<E> for Result<(), E> {
    fn log(self) -> () {
        match self {
            Ok(()) => (),
            Err(e) => eprintln!("{:?}", e)
        }
    }
}