#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_io::Read;
use mb9bf61xt;

mod serial;
pub use crate::serial::Mb9bf61xtUart;

const INPUT_LINE_LENGTH: usize = 64;

fn disable_wdg() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let wdg = p.HWWDT;

    // Unlock the watchdog registers.
    wdg.wdg_lck().write(|w| unsafe { w.bits(0x1ACCE551) });
    wdg.wdg_lck().write(|w| unsafe { w.bits(0xE5331AAE) });

    // Disable the hardware watchdog timer.
    wdg.wdg_ctl().modify(|_, w| w.inten().clear_bit());
}

fn init_clock() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let clock = p.CRG;

    // Force-clear all possible oscillization stabilization wait interrupt causes (main, sub, and PLL).
    clock.int_clr().write(|w| unsafe { w.bits(0x7) });

    // Set the main oscillator stabilization wait time (use default ~500ns).
    clock.csw_tmr().write(|w| unsafe { w.bits(0) });

    // Set the oscillation stabilization wait interrupt.
    clock.int_enr().modify(|_, w| w.mcse().set_bit());

    // Enable the main (external) clock oscillator.
    clock.scm_ctl().modify(|_, w| w.mosce().set_bit());

    // Checkk the main oscillator stabile bit.
    // TODO: anything to be done with a time-out here?
    while clock.scm_str().read().mordy() != true {}

    // Configure the master clock to use the main (external clock).
    clock.scm_ctl().modify(|_, w| unsafe { w.rcs().bits(0x1) });

    // Wait for master clock selection to take effect.
    // TODO: anything to be done with a time-out here?
    while clock.scm_str().read().rcm() != 0x1 {}
}

fn init_pins() {
    // FM3 (MB9BF61xT) UART pin options:
    // NOTE: UART channels 0-3 don't offer a FIFO, channels 4-7 do have a FIFO.
    //
    // * SIN0_0:P21, SIN1_0:P56, SIN2_0:P72, SIN3_0:P75, SIN4_0:PD2, SIN5_0:P60, SIN6_0:P53, SIN7_0:P59
    // * SOT0_0:P22, SOT1_0:P57, SOT2_0:P73, SOT3_0:P76, SOT4_0:PD1, SOT5_0:P61, SOT6_0:P54, SOT7_0:P5A
    //
    // This code uses UART channel 4.  On the CQ-FRK-FM3 board this uses connector CN2's pin 14 (SOT 4_2,
    // GPIO P06) and pin 11 (SIN 4_2, GPIO P05).
    //

    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let gpio = p.GPIO;

    // Port Function Setting register (PFRx) - configure channel 4's UART pins to be a peripheral (not GPIO) function.
    // PFR0 controls GPIO pins P00 through P0F.  Configure P05 (SIN 4_2) and P06 (SOT 4_2) as peripheral pins.
    gpio.pfr0().modify(|_, w| w.p05().set_bit());
    gpio.pfr0().modify(|_, w| w.p06().set_bit());

    // Extended Pin Function Setting register (EPFR0x) - configure UART pin functions.
    // EPFR08 controls UART channels 4-7.
    let mut current_epfr08 = gpio.epfr08().read().bits();
    current_epfr08 &= !0x3FF; // Mask off channel 4 bits.
    current_epfr08 |= 0x0FC; // No RTS, CTS 4_2, SIN 4_2, SOT 4_2, no SCK.
    gpio.epfr08().write(|w| unsafe { w.bits(current_epfr08) });
}

fn print_banner() {
    println!("");
    println!("  ____            _       _   __  __             _ _             ");
    println!(" / ___|  ___ _ __(_) __ _| | |  \\/  | ___  _ __ (_) |_ ___  _ __ ");
    println!(" \\___ \\ / _ \\ '__| |/ _` | | | |\\/| |/ _ \\| '_ \\| | __/ _ \\| '__|");
    println!("  ___) |  __/ |  | | (_| | | | |  | | (_) | | | | | || (_) | |   ");
    println!(" |____/ \\___|_|  |_|\\__,_|_| |_|  |_|\\___/|_| |_|_|\\__\\___/|_|  ");
    println!("");
}

fn print_command_list() {
    println!(" COMMANDS:");
    println!(" ------------------------------------------------------------------------------------------------------------------");
    println!(" <address>                       = displays the contents of a single address");
    println!(" .<address>                      = displays the contents between the last opened location and the specified address");
    println!(
        " <address1>.<address2>           = displays the contents between address1 and address2"
    );
    println!(" <address1> <address2> <...>     = diplays the contents of address1, address2, ...");
    println!(
        " <address>:<data>                = writes the specified data (as u32) to the specified address"
    );
    println!(" <address>:<data1> <data2> <...> = writes data1, data2, ... starting from the specified address");
    println!(
        " <address> R                     = executes code starting from the address specified"
    );
    println!("");
}

#[entry]
fn main() -> ! {
    // Disable the hardware watchdog timer.
    disable_wdg();

    // Initialize the master clock to use the main (external) clock.
    init_clock();

    // Initialize UART pins.
    init_pins();

    // Initialize the UART controller.
    let mut uart4 = Mb9bf61xtUart::new();
    uart4.init_uart();

    // Print banner and command list.
    print_banner();
    println!(" version 0.1, Jeff Glaum <jeffglaum@live.com>");
    println!("");
    print_command_list();

    let mut line_buf: [u8; INPUT_LINE_LENGTH] = [0; INPUT_LINE_LENGTH];

    loop {
        print!("> ");

        // Read a full line from the UART.
        let mut i = 0;
        loop {
            let _bytes_read = uart4.read(&mut line_buf[i..]);
            // TODO: handle both CR and LF anywhere within the slice.
            if line_buf[i] == b'\n' {
                break;
            }
            i += _bytes_read.unwrap();
            if i >= INPUT_LINE_LENGTH {
                i = INPUT_LINE_LENGTH;
                break;
            }
        }

        // Line buffer may have BS characters in it, process those and remove non-printable characters.
        clean_input(&mut line_buf);

        let s = core::str::from_utf8(&line_buf[0..i]).unwrap();

        // Process the command line.
        process_cmdline(s);
    }
}

// TODO: how to get rid of buffer length here?
fn clean_input(buf: &mut [u8; INPUT_LINE_LENGTH]) {
    let len = buf.len();
    let mut a = 0;
    let mut b = 0;

    // Process the string "in place" to remove backspace characters.
    while a < len && b < len {
        // Look for BS.
        if buf[b] == 0x08 {
            if a >= 2 {
                a -= 2;
            };
        } else {
            buf[a] = buf[b];
        }
        // Filter out any non-printable characters.
        if buf[b] >= 32 || buf[b] <= 126 {
            a += 1;
        }
        b += 1;
    }

    // Zero-fill the remainder of the buffer.
    while a < len {
        buf[a] = 0;
        a += 1;
    }
}

// Command line parser tokens.
enum Tokens {
    EOF,
    Number,
    SeparatorDot,
    SeparatorColon,
    CommandRun,
}

// Command line processing command states.
enum States {
    None,
    SetAddress,
    DisplayRange,
    WriteDataAtAddress,
    Invalid,
}

fn _print_tokens(index: usize, val: u32, token: &Tokens) {
    if matches!(token, Tokens::Number) {
        println!("Token: Number, Value: 0x{:x}, Index: {}", val, index);
    } else if matches!(token, Tokens::SeparatorColon) {
        println!("Token: SeparatorColon, Index: {}", index);
    } else if matches!(token, Tokens::SeparatorDot) {
        println!("Token: SeparatorDot, Index: {}", index);
    } else if matches!(token, Tokens::CommandRun) {
        println!("Token: CommandRun, Index: {}", index);
    } else if matches!(token, Tokens::EOF) {
        println!("Token: EOF, Index: {}", index);
    } else {
        println!("ERROR: unknown token.");
    }
}

// Last address opened.
static mut OPENED_ADDRESS: u32 = 5;

fn process_cmdline(s: &str) {
    let mut i = 0;
    let mut current_state: States = States::None;

    // Commands mimic the Apple 1 Monitor (a.k.a. Wozmon).
    //
    // <address>                       = displays the contents of a single address (and sets the opened location).
    // .<address>                      = displays the contents between the last opened location and the specified end address.
    // <address1>.<address2>           = displays the contents between address1 and address2.
    // <address1> <address2> <...>     = diplays the contents of address1, address2, ...
    // <address>:<data>                = writes data (as u32) to the specified address.
    // <address>:<data1> <data2> <...> = writes data1, data2, ... starting from the address specified.
    // <address> R                     = executes code starting from the address specified.
    //
    loop {
        let (index, val, token) = get_next_token(s, i).unwrap();

        // For debugging...
        //_print_tokens(index, val, &token);

        // Command parser state machine.
        //
        if matches!(current_state, States::None) {
            // *** Starting state ***
            if matches!(token, Tokens::SeparatorDot) {
                // Display range from last opened address.
                current_state = States::DisplayRange;
            } else if matches!(token, Tokens::Number) {
                // Open a new address.
                unsafe {
                    OPENED_ADDRESS = val;
                }
                current_state = States::SetAddress;
            } else {
                current_state = States::Invalid;
            }
        } else if matches!(current_state, States::SetAddress) {
            // *** Addressed opened ***
            if matches!(token, Tokens::SeparatorDot) {
                // Display range from last opened address.
                current_state = States::DisplayRange;
            } else if matches!(token, Tokens::SeparatorColon) {
                // Write data value at last opened address.
                current_state = States::WriteDataAtAddress;
            } else if matches!(token, Tokens::CommandRun) {
                // Execute from address.
                execute_address(unsafe { OPENED_ADDRESS });
                current_state = States::None;
            } else if matches!(token, Tokens::Number) {
                // Display another address.
                display_address(unsafe { OPENED_ADDRESS });
                unsafe {
                    OPENED_ADDRESS = val;
                }
                current_state = States::SetAddress;
            } else if matches!(token, Tokens::EOF) {
                // Display address.
                display_address(unsafe { OPENED_ADDRESS });
                current_state = States::None;
            } else {
                current_state = States::Invalid;
            }
        } else if matches!(current_state, States::DisplayRange) {
            // *** Display range ***
            if matches!(token, Tokens::Number) {
                // Display a range of addresses.
                display_address_range(unsafe { OPENED_ADDRESS }, val);
                current_state = States::None;
            }
        } else if matches!(current_state, States::WriteDataAtAddress) {
            // *** Write data to opened address ***
            if matches!(token, Tokens::Number) {
                // Write one or more data values.
                write_data_to_address(unsafe { OPENED_ADDRESS }, val);
                // Automatically increment to next address in order.
                unsafe {
                    OPENED_ADDRESS += 4;
                } // Size of u32.
                current_state = States::WriteDataAtAddress;
            } else {
                current_state = States::Invalid;
            }
        }

        // Done?
        if matches!(token, Tokens::EOF) {
            break;
        }

        // Capture the last index for the next tokenizer iteration.
        i = index;
    }
}

fn display_address_range(a: u32, b: u32) {
    // Make sure the end address comes after the start address.
    if b < a {
        println!(
            "ERROR: Start address 0x{:x} comes after end address 0x{:x}.",
            a, b
        );
        return;
    }

    let n = (b - a) + 1;
    let mut show_address: bool = true;
    for i in 0..n {
        if (a + i) % 16 == 0 {
            println!("");
            show_address = true;
        }

        if show_address {
            print!("{:08X}: ", (a + i));
            show_address = false;
        }

        // Display contents of memory.
        let p = (a + i) as *const u8;
        let n = unsafe { core::ptr::read(p) };
        print!("{:02X} ", n);
    }
    println!("");
}

fn write_data_to_address(a: u32, d: u32) {
    // Write data to a specific address.
    let p = a as *mut u32;
    unsafe { core::ptr::write(p, d) };
}

fn display_address(a: u32) {
    print!("{:08X}: ", a);

    // Display contents of memory.
    let p = a as *const u8;
    let n = unsafe { core::ptr::read(p) };
    println!("{:02X} ", n);
}

fn execute_address(a: u32) {
    let func_ptr = a as *const u32;

    println!("Jumping to address 0x{:08X}...", a);
    unsafe {
        let func: extern "C" fn() = core::intrinsics::transmute(func_ptr);
        func();
    }
}

fn iswhitespace(c: char) -> bool {
    if c == ' ' || c == '\t' || c == '\r' || c == '\n' {
        return true;
    } else {
        return false;
    }
}

// TODO: how to handle returned values?
fn get_next_token(s: &str, i: usize) -> Result<(usize, u32, Tokens), core::convert::Infallible> {
    for x in i..s.len() {
        let c = s.chars().nth(x).unwrap();

        // Skip leading whitespace and other such characters.
        if iswhitespace(c) {
            continue;
        }

        // Check for dot.
        if c == '.' {
            return Ok((x + 1, 0, Tokens::SeparatorDot));
        }

        // Check for colon.
        if c == ':' {
            return Ok((x + 1, 0, Tokens::SeparatorColon));
        }

        // Check for command - currently only one (Run).
        if c == 'R' || c == 'r' {
            return Ok((x + 1, 0, Tokens::CommandRun));
        }

        // Must be a number - extract the slice and convert it.
        for y in x..s.len() {
            let c2 = s.chars().nth(y).unwrap();
            if !iswhitespace(c2) && c2 != '.' && c2 != ':' {
                continue;
            }

            let without_prefix = s[x..y].trim_start_matches("0x");
            let num = u32::from_str_radix(without_prefix, 16).unwrap();

            //let num = s[x..y].parse::<usize>().unwrap();
            return Ok((y, num, Tokens::Number));
        }
    }

    return Ok((0, 0, Tokens::EOF));
}
