# Rytmos PICO

A collection of crates with the top level software for PCB's in `pcbs/`, and libraries for common code.

## Tooling setup
```shell
rustup target add thumbv6m-none-eabi
cargo install flip-link
yay -S cargo-binstall # Or whatever package manager you prefer
cargo binstall probe-rs-tools
curl https://probe.rs/files/69-probe-rs.rules | sudo tee /etc/udev/rules.d/69-probe-rs.rules
sudo udevadm control --reload
sudo udevadm trigger
```
Now you can build and flash using `cargo run` in the corresponding subdirectory.
