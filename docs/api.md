# RTK Engine C-API Documentation

The RTK Engine provides a stable C interface (`rtk_api.h`) for integration with non-C++ environments (e.g., Python, C#, Rust).

## API Overview

```c
#include "rtk_api.h"
```

### Types

- `typedef void* rtk_handle_t;`
  - An opaque pointer representing an instance of the RTK Engine.

### Functions

#### `rtk_handle_t rtk_init(const char* config_path);`
- **Description:** Initializes the RTK engine using the provided TOML configuration file.
- **Parameters:** `config_path` (const char*) - Filesystem path to the TOML configuration file.
- **Returns:** A handle to the initialized engine, or `NULL` on failure.

#### `int rtk_process_epoch(rtk_handle_t handle, const char* data);`
- **Description:** Processes a single epoch of raw GNSS observations provided as a character string (e.g., serialized JSON or specialized binary-encoded data).
- **Parameters:**
  - `handle` (rtk_handle_t) - The engine instance.
  - `data` (const char*) - The raw data string.
- **Returns:** `0` on success, non-zero error code otherwise.

#### `void rtk_shutdown(rtk_handle_t handle);`
- **Description:** Shuts down the engine and releases all associated resources.
- **Parameters:** `handle` (rtk_handle_t) - The engine instance.

## Usage Example

```c
rtk_handle_t engine = rtk_init("config.toml");
if (engine) {
    rtk_process_epoch(engine, raw_obs_data);
    rtk_shutdown(engine);
}
```
