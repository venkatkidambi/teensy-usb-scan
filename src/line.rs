//! Fixed-capacity newline framing shared by the firmware's ASCII protocols.
//! Overflow discards the whole line through its newline, then reception resumes.
pub struct LineBuffer<const N: usize> {
    bytes: [u8; N],
    len: usize,
    dropping: bool,
}

impl<const N: usize> LineBuffer<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            dropping: false,
        }
    }

    pub fn push(&mut self, byte: u8) -> Option<Result<&[u8], ()>> {
        if byte == b'\n' {
            let len = self.len;
            self.len = 0;
            return Some(if core::mem::replace(&mut self.dropping, false) {
                Err(())
            } else {
                Ok(&self.bytes[..len])
            });
        }
        if !self.dropping {
            if self.len == N {
                self.dropping = true;
            } else {
                self.bytes[self.len] = byte;
                self.len += 1;
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capacity_overflow_and_recovery() {
        let mut lines = LineBuffer::<3>::new();
        for byte in b"abc" {
            assert!(lines.push(*byte).is_none());
        }
        assert_eq!(lines.push(b'\n'), Some(Ok(&b"abc"[..])));
        for byte in b"abcdignored" {
            assert!(lines.push(*byte).is_none());
        }
        assert_eq!(lines.push(b'\n'), Some(Err(())));
        assert_eq!(lines.push(b'\n'), Some(Ok(&b""[..])));
        assert!(lines.push(b'\r').is_none());
        assert_eq!(lines.push(b'\n'), Some(Ok(&b"\r"[..])));
    }
}
