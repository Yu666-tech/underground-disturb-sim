# Third-party provenance

SubT source reference: https://github.com/osrf/subt , branch `fortress-noetic`, commit `ba15f9ff35ccfb43f44937d6d84e047615d174ff`.

This project reuses environment resources; it does not run or redistribute the SubT ROS 1 control stack.

The ROS 2 scene assembly and trajectory plugin target Gazebo Classic.

Pinned OpenRobotics Fuel resources:

| Local ID                 | Fuel model          | Version |
| ------------------------ | ------------------- | ------- |
| subt_tunnel_tile_1       | Tunnel Tile 1       | 5       |
| subt_tunnel_tile_2       | Tunnel Tile 2       | 16      |
| subt_tunnel_tile_5       | Tunnel Tile 5       | 14      |
| subt_tunnel_tile_blocker | Tunnel Tile Blocker | 12      |

`asset_manifest.json` retains model `authorship/source` identifiers and the hashes of the 117 archived files. `asset_sources.json` records pinned file URLs, donor texture origins and the COLLADA image-path replacements required to reproduce the localized archive.

Mesh geometry is preserved; paths and `material` names are localized, PBR blocks unsupported by Classic are removed, shared textures are resolved, and competition-only plugins are omitted.

Large `mesh/texture` files are fetched from their original source or imported from a local archive rather than rehosted here.

Model authorship remains in `model.config`.

Third-party resources remain subject to their original terms; this repository does not apply a blanket license to third-party resources.

No project-wide open-source license has been selected for the original platform code yet.
