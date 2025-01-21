# How to build

Set up virtualenv:
```sh
pyenv virtualenv gym_env
pyenv local gym_env
```

Activate virtualenv:
```
pyenv activate
```

Build using maturing (release flags are optional):
```sh
cd crates/environment
maturin develop --release # builds the crate and installs it as a python module directly in the current virtualenv
maturin build --release # builds the crate, stores it in target/wheel
```

