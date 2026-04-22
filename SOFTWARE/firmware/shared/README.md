# Firmware Shared

This folder is reserved for embedded code that is reused across multiple sketches or controller targets.

Good candidates for this area include:

- reusable patch geometry helpers
- common serial parsing code for `CAR,...`, `ERR,...`, or patch-upload protocols
- LED animation or segment-management utilities
- shared hardware abstraction layers
- configuration structs or constants used by both experiments and systems

The current codebase is still organized primarily by experiment and system folder, so no shared firmware modules are committed here yet. When duplication starts appearing across sketches, this should be the first place to consolidate it.
