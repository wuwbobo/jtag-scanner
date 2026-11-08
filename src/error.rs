use std::{
    cell::{BorrowError, BorrowMutError},
    fmt::Display,
    panic::Location,
};

/// The error type used in this crate.
#[derive(Debug, Clone)]
pub struct Error {
    kind: ErrorKind,
    message: String,
    location: &'static Location<'static>,
}

impl Error {
    #[track_caller]
    #[allow(unused)]
    pub(crate) fn new<S: ToString>(kind: ErrorKind, message: S) -> Self {
        Error {
            kind,
            message: message.to_string(),
            location: Location::caller(),
        }
    }

    #[track_caller]
    pub(crate) fn result<T, S: ToString>(kind: ErrorKind, message: S) -> Result<T, Self> {
        Err(Error {
            kind,
            message: message.to_string(),
            location: Location::caller(),
        })
    }

    /// Returns the corresponding [`ErrorKind`] for this error.
    pub fn kind(&self) -> ErrorKind {
        self.kind
    }

    /// Returns the message for this error.
    pub fn message(&self) -> &str {
        &self.message
    }

    /// Where the error is produced in the `jtag_scanner` crate.
    pub fn location(&self) -> &'static Location<'static> {
        self.location
    }
}

impl Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for Error {}

/// A list of general categories of errors produced in this crate.
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub enum ErrorKind {
    /// Cannot get IR codes or pin IDs for a TAP without an attached BSDL.
    BsdlUnavailable,
    /// Bad JTAG connection.
    BrokenChain,
    /// Pin group has been invalidated by an ended session.
    SessionEnded,
    /// The pin does not support the operation because of its type.
    WrongPinType,
    /// Conflicting with an existing group or group item.
    GroupingConflict,
    /// Pin operation beyond the limits of the pin group.
    GroupLimit,
    /// Other error.
    Other,
}

impl From<&str> for Error {
    #[track_caller]
    fn from(value: &str) -> Self {
        Error {
            kind: ErrorKind::Other,
            message: value.to_string(),
            location: Location::caller(),
        }
    }
}

impl From<String> for Error {
    #[track_caller]
    fn from(value: String) -> Self {
        Error {
            kind: ErrorKind::Other,
            message: value,
            location: Location::caller(),
        }
    }
}

// `BorrowError` and `BorrowMutError` should not occur in the current crate,
// but convert them in case of something gets wrong.

impl From<BorrowError> for Error {
    #[track_caller]
    fn from(_: BorrowError) -> Self {
        Error {
            kind: ErrorKind::Other,
            message: "scanner is being used by another pin group".into(),
            location: Location::caller(),
        }
    }
}

impl From<BorrowMutError> for Error {
    #[track_caller]
    fn from(_: BorrowMutError) -> Self {
        Error {
            kind: ErrorKind::Other,
            message: "scanner is being used by another pin group".into(),
            location: Location::caller(),
        }
    }
}
