//! Module of general traits.
use std::sync::{Mutex, MutexGuard};

/// Provides [LockOrUnpoison::lock_no_poison], which allows
/// a lock primitive to infinitely attempt to lock the primitive
/// until un-poisoned.
pub trait LockOrUnpoison<T> {
    /// Attempts to acquire a lock of the synchronization primitive.
    /// If the locking failed due to a poison (some thread panicked while mutex was locked),
    /// clear the poisoned state and try again.
    fn lock_no_poison(&self) -> MutexGuard<'_, T>;
}

/// Implementation of the no poison lock for the existing [`Mutex`].
/// This method will block until poison is cleared.
impl<T> LockOrUnpoison<T> for Mutex<T> {
    fn lock_no_poison(&self) -> MutexGuard<'_, T> {
        loop {
            if let Ok(guard) = self.lock() {
                return guard;
            }
            // Mutex was poisoned => un-poison
            self.clear_poison();
        }
    }
}
