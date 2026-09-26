# Contributing to bullet3-rs

Thank you for your interest in contributing to `bullet3-rs`! We welcome contributions ranging from bug reports and documentation fixes to performance optimizations, constraint formulations, and new collision shapes.

---

## Code of Conduct

All contributors and maintainers are expected to adhere to our [Code of Conduct](CODE_OF_CONDUCT.md). Please read it to understand our community standards.

---

## Development Setup

`bullet3-rs` is written in pure Rust with zero C/C++ dependencies. You will need:
- Rust toolchain (`stable` or 1.80+)
- `cargo`, `rustfmt`, and `clippy`

### Clone and Build

```bash
git clone https://github.com/bhubbard/bullet3-rs.git
cd bullet3-rs
cargo build
```

---

## Running Tests and Demos

Always verify that the test suite and CLI demonstrations pass before opening a pull request:

```bash
# Run unit & integration tests
cargo test

# Run interactive simulation demos
cargo run -- drop
cargo run -- stack
cargo run -- pendulum
cargo run -- billiards
cargo run -- benchmark
```

---

## Pull Request Guidelines

1. **Keep Pull Requests Focused**: Limit changes to a single feature or bug fix.
2. **Determinism and Stability**: Ensure constraints and collision responses maintain numerical stability under standard integration timesteps (e.g. 60 Hz).
3. **Format and Lint**: Run `cargo fmt` and `cargo clippy` before submitting.
4. **Preserve Compatibility**: Keep the public API clean and idiomatic Rust while preserving compatibility with Bullet Physics conventions where beneficial.
