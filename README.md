# Set up db

```sh
cargo sqlx migrate run --source migrations
```

# Revert db

```sh
cargo sqlx migrate revert --source migrations
```
