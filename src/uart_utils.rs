pub struct UARTBuffer {
    pub data: [u8; 512],
    pub head: usize,
    pub tail: usize,
}

impl UARTBuffer {
    pub fn push(&mut self, item: u8) {
        self.data[self.head] = item;

        // Move head to the next position
        self.head = (self.head + 1) % self.data.len();

        // If head meets tail, it means the buffer is full; move tail to the next position
        if self.head == self.tail {
            self.tail = (self.tail + 1) % self.data.len();
        }
    }

    #[allow(dead_code)]
    pub fn clear_buffer(&mut self) {
        self.head = 0;
        self.tail = 0;
        for byte in self.data.iter_mut() {
            *byte = 0;
        }
    }

    pub fn lines(&self) -> heapless::Vec<heapless::String<64>, 32> {
        let mut result: heapless::Vec<heapless::String<64>, 32> = heapless::Vec::new();

        critical_section::with(|_| {
            for line in self.data.split(|&byte| byte == b'\n') {
                if !line.is_empty() {
                    if let Ok(string) = {
                        let line_vec = match heapless::Vec::from_slice(line) {
                            Ok(v) => v,
                            Err(_) => continue,
                        };
                        heapless::String::<64>::from_utf8(line_vec)
                    } {
                        result.push(string).ok(); // Handle potential overflow
                    }
                }
            }
        });

        result
    }
}
