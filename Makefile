.PHONY: build-s3 build-c3 build-c6 run-s3 run-c3 run-c6 clippy-s3 clippy-c3 clippy-c6

CLIPPY_ARGS = --workspace -- -D warnings

build-s3:
	ln -sf rust-toolchain-xtensa.toml rust-toolchain.toml
	cargo build --release --target xtensa-esp32s3-none-elf --no-default-features --features esp32s3 $(ARGS)

build-c3:
	ln -sf rust-toolchain-riscv.toml rust-toolchain.toml
	cargo build --release --target riscv32imc-unknown-none-elf --no-default-features --features esp32c3 $(ARGS)

build-c6:
	ln -sf rust-toolchain-riscv.toml rust-toolchain.toml
	cargo build --release --target riscv32imac-unknown-none-elf --no-default-features --features esp32c6 $(ARGS)

run-s3:
	ln -sf rust-toolchain-xtensa.toml rust-toolchain.toml
	cargo run --release --target xtensa-esp32s3-none-elf --no-default-features --features esp32s3 $(ARGS)

run-c3:
	ln -sf rust-toolchain-riscv.toml rust-toolchain.toml
	cargo run --release --target riscv32imc-unknown-none-elf --no-default-features --features esp32c3 $(ARGS)

run-c6:
	ln -sf rust-toolchain-riscv.toml rust-toolchain.toml
	cargo run --release --target riscv32imac-unknown-none-elf --no-default-features --features esp32c6 $(ARGS)

clippy-s3:
	ln -sf rust-toolchain-xtensa.toml rust-toolchain.toml
	cargo clippy --target xtensa-esp32s3-none-elf --no-default-features --features esp32s3 $(CLIPPY_ARGS)

clippy-c3:
	ln -sf rust-toolchain-riscv.toml rust-toolchain.toml
	cargo clippy --target riscv32imc-unknown-none-elf --no-default-features --features esp32c3 $(CLIPPY_ARGS)

clippy-c6:
	ln -sf rust-toolchain-riscv.toml rust-toolchain.toml
	cargo clippy --target riscv32imac-unknown-none-elf --no-default-features --features esp32c6 $(CLIPPY_ARGS)

clean:
	cargo clean
