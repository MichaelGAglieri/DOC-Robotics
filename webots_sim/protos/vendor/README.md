# Cyberbotics sources

UR3e_R2025a.proto is the original Cyberbotics UR3e PROTO, Apache-2.0. DocArm5 derives from it by removing the sixth joint/link and changing the tool mount, palette and local resource paths. The original header is retained.

Mesh, texture and HDR assets are not redistributed here. Their R2025a URLs and SHA-256 digests are recorded in webots/manifest.json. Run `pixi run assets` once to retrieve them. After setup the world resolves these resources locally. Asset notices remain those of the original sources; see THIRD_PARTY.md and LICENSES/Apache-2.0.txt at the repository root.
