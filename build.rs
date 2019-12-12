use std::env;
use std::fs::{read_dir, File};
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::PathBuf;

#[derive(Debug, PartialEq, PartialOrd, Eq, Ord)]
struct CartHdr {
    name: Vec<u8>,
    year: u32,
    path: PathBuf,
}

fn parse_cart_header(path: &PathBuf) -> Option<CartHdr> {
    if let Ok(file) = File::open(path) {
        let mut r = BufReader::new(file);
        let mut copyright = Vec::new();
        let mut name = Vec::new();
        if r.read_until(0x80, &mut copyright).is_err()
            || copyright.len() < 5
            || &copyright[0..=1] != b"g "
        {
            println!(
                "cargo:warning={:?}: not a cart (© = {:?})?",
                path,
                String::from_utf8_lossy(&copyright)
            );
            return None;
        }

        std::io::copy(&mut r.by_ref().take(6), &mut std::io::sink()).unwrap(); // Skip next six bytes
        if r.take(64).read_until(0x80, &mut name).is_err() || name.len() < 1 {
            println!(
                "cargo:warning={:?}: not a cart (name = {:?})?",
                path,
                String::from_utf8_lossy(&name)
            );
            return None;
        }
        name.pop().unwrap();

        let year = &copyright[copyright.len() - 5..copyright.len() - 1];
        if let Ok(year) = String::from_utf8_lossy(&year).parse() {
            return Some(CartHdr {
                path: path.clone(),
                year,
                name,
            });
        }
    }
    None
}

const FILEDATA: u16 = 0x400; // From multicart.asm
const MAX_NAME_LEN: usize = 18; // With default font used in multicart.asm

fn make_cart_table(carts: &Vec<CartHdr>) -> Vec<u8> {
    let max_len = carts
        .iter()
        .map(|c| c.name.len())
        .max()
        .unwrap()
        .min(MAX_NAME_LEN);

    let pointer_list_len = ((carts.len() + 1) * std::mem::size_of::<u16>()) as u16;
    let mut string_ptr = FILEDATA + pointer_list_len;
    let mut pointers: Vec<u8> = Vec::new();
    let mut strings: Vec<u8> = Vec::new();
    for cart in carts {
        pointers.extend_from_slice(&string_ptr.to_be_bytes());

        let mut string = cart.name.clone();
        while string.len() < max_len {
            string.push(b' ');
        }
        while string.len() > max_len {
            string.pop().unwrap();
        }
        string.extend_from_slice(&format!(" {}", cart.year).as_bytes());
        string.push(0x80); // String terminator

        /*
        println!(
            "cargo:warning={:04x}: {:?} ({})",
            string_ptr,
            String::from_utf8_lossy(&string),
            cart.year
        );
        */

        string_ptr += string.len() as u16;
        strings.append(&mut string);
    }
    pointers.push(0x00); // String list terminator
    pointers.push(0x00);
    pointers.append(&mut strings);

    pointers
}

fn main() {
    // Put the linker script somewhere the linker can find it
    let out_dir = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out_dir.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out_dir.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    println!("cargo:rerun-if-changed=memory.x");

    // rebuild multicart.bin if changed
    println!("cargo:rerun-if-changed=multicart/multicart.asm");
    match std::process::Command::new("make")
        .args(&["-C", "multicart"])
        .status()
    {
        Err(e) => println!("cargo:warning=failed to rebuild multicart: {:?}", e),
        _ => (),
    }

    // Read carts directory
    let root_dir = &PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("carts");
    let mut carts: Vec<CartHdr> = read_dir(&root_dir)
        .expect("carts directory:")
        .flatten()
        .map(|e| parse_cart_header(&e.path()))
        .flatten()
        .collect();

    // Rerun if carts directory changes
    println!("cargo:rerun-if-changed=carts");

    // Sort by name,year,path
    carts.sort();

    // Generate multicart loader with the table
    let cart_table = make_cart_table(&carts);
    let multicart = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/multicart/multicart.bin"
    ));
    let cart_path = out_dir.join("loader.bin");
    let mut cart_file = File::create(&cart_path).unwrap();
    cart_file
        .write_all(&multicart[0..FILEDATA as usize])
        .unwrap();
    cart_file.write_all(&cart_table).unwrap();

    // Generate code for embedding carts in stm flash
    let source_path = out_dir.join("carts.rs");
    let source_file = File::create(&source_path).unwrap();
    let mut writer = BufWriter::new(&source_file);
    writeln!(
        &mut writer,
        "const CARTS: [*const u8; {}] = [\n",
        carts.len()
    )
    .unwrap();
    for cart in carts {
        writeln!(
            &mut writer,
            "include_bytes!(\"{}\") as *const u8,\n",
            cart.path.to_str().unwrap(),
        )
        .unwrap();
    }
    writeln!(&mut writer, "];\n").unwrap();
}
