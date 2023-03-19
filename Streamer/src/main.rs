extern crate repng;
extern crate scrap;

use scrap::{Capturer, Display};
use std::io::ErrorKind::WouldBlock;
use std::thread;
use std::time::Duration;

use serial2::SerialPort;

fn pritntBuffer(buffer: &[u8]){
    for i in 0..buffer.len() {
        if i % 320 == 0 {
            println!("");
        }
        // print the binary representation of the value
        // print!("{:08b}", buffer[i]);
        if(buffer[i] > 50){
            print!("1");
        }else{
            print!("0");
        }
    }
    println!("");
}

fn main() {
    let target_x_size = 320;
    let target_y_size = 240;

    let x_offset = 0;
    let y_offset = 0;

    let threshold = 80;
    let smoothing = false;

    let magic:char = 'A';

    //let x_step = (w as f32 / target_x_size as f32).floor() as usize;
    let x_step = 1;
    let y_step = x_step;

    let port = SerialPort::open("/dev/ttyACM1", 115200);
    let mut ser_port;
    match port {
        Ok(port) => {
            println!("Port opened");
            ser_port = port;
        }
        Err(e) => {
            println!("Error opening port: {}", e);
            // exit the program
            return;
        }
    }

    let one_second = Duration::new(1, 0);
    let one_frame = one_second / 60;

    let display = Display::primary().expect("Couldn't find primary display.");
    let mut capturer = Capturer::new(display).expect("Couldn't begin capture.");
    let (w, h) = (capturer.width(), capturer.height());

    println!("x_step: {}, y_step: {}", x_step, y_step);
    println!("Window size: {}, {}", x_step*target_x_size, y_step*target_y_size);
    //thread::sleep(Duration::from_millis(2000));

    // a buffer of grayscale values
    let mut target_buffer:Vec<u8> = vec![0; target_x_size * target_y_size];    

    let mut last_loop = std::time::Instant::now();
    loop {

        // Wait until there's a frame.
        let buffer = match capturer.frame() {
            Ok(buffer) => buffer,
            Err(error) => {
                if error.kind() == WouldBlock {
                    thread::sleep(one_frame);
                    continue;
                } else {
                    panic!("Error: {}", error);
                }
            }
        };

        println!("Captured! tdelta: {}us", std::time::Instant::now().duration_since(last_loop).as_micros());
        last_loop = std::time::Instant::now();

        let stride = buffer.len() / h;

        let mut iter = 0;
        for y in 0..target_y_size {
            for x in 0..target_x_size {
                // convert coordinates from the target coordinates to the screen coordinates
                let bigX = x * x_step + x_offset;
                let bigY = y * y_step + y_offset;
                // check if the coordinates are valid and aren't outside the screen
                if bigX >= w || bigY >= h {
                    continue;
                }

                let mut pixel:u8 = 0;
                if smoothing {
                    let mut accumulator = 0;
                    for i in 0..x_step {
                        for j in 0..y_step {
                            let i = stride * (bigY + j) + 4 * (bigX + i);
                            let r = buffer[i + 2];
                            let g = buffer[i + 1];
                            let b = buffer[i];
                            accumulator += (r as u16 + g as u16 + b as u16)/3;
                        }
                    }
                    let avg = accumulator / (x_step * y_step) as u16;
                    pixel = avg as u8;
                }else{
                    let i = stride * bigY + 4 * bigX;
                    let r = buffer[i + 2];
                    let g = buffer[i + 1];
                    let b = buffer[i];
                    let avg = (r as u16 + g as u16 + b as u16)/3;
                    pixel = avg as u8;
                    //println!("Reading: X: {}, Y: {}, STRIDE: {}; R: {}, G: {}, B: {} - {} ", bigX, bigY, stride, r, g, b, pixel);
                }
                
                target_buffer[iter] = pixel;
                iter += 1;
            }
        }
        
        // pritntBuffer(&target_buffer);

        // now we create the u8 buffer to be sent
        let mut data_buffer:Vec<u8> = vec![0; target_buffer.len() / 4];
        // iterate in 16 bit chunks
        for i in 0..target_buffer.len()/4 {
            // threshold represents the maximum brightness value
            // so if a pixel is > threshold, we set it to 11
            // if it's threshold/2, we set it to 10
            // if it's 0, we set it to 00

            // byte structure:
            // 00 00 00 00 - 1 byte
            // each collection of 2 bits is a brightness value
            // 00 = black
            // 01 = 1 - 50% in our case
            // 10 || 11 = 2 - 100% in our case

            let mut data:Vec<u8> = vec![0; 4];

            for j in 0..4 {
                let pixel = target_buffer[i*4 + j];
                if pixel > threshold {
                    data[j] = 2;
                }else if pixel > threshold/2 {
                    data[j] = 1;
                }else{
                    data[j] = 0;
                }
            }

            // pack the data into a byte
            let mut byte:u8 = 0;
            for j in 0..4 {
                byte |= data[j] << (6 - j*2);
            }

            data_buffer[i] = byte;
            // print the byte
            // println!("{}: {:08b}", i, byte);
        }

        loop {
            //ser_port.flush();
            let mut buffer = [0; 1];
            let read = ser_port.read(&mut buffer);
            match read {
                Ok(read) => {
                    // if we read a magic char, we can send the data
                    if buffer[0] as char == magic {
                        ser_port.write(&data_buffer);
                        break;
                    }
                }
                Err(e) => {
                    continue;
                }
            }
        }

        //break;
    }
}
