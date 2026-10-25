# Third-party material and license boundaries

Original DOC-specific code in this release: GPL-3.0-only, Michael Giuseppe Aglieri. This applies to source files authored for this project; it does not relicense external components.

| Component | Author/source | Treatment in this release |
| --- | --- | --- |
| UR3e R2025a PROTO | [Cyberbotics/Webots](https://github.com/cyberbotics/webots/blob/R2025a/projects/robots/universal_robots/protos/UR3e.proto) | Original copy in vendor and derived DocArm5 keep Apache-2.0; see LICENSES/Apache-2.0.txt |
| DocArm5 model adaptation | DOC project, based on Cyberbotics | Five joints, removed terminal joint/link, changed tool mount, local paths and palette; retains the original Apache notice |
| Meshes, HDR and appearance resources | Cyberbotics/Webots R2025a | Not redistributed here; official URLs and SHA-256 in the manifest, setup script retrieves them for use with Webots |
| Physical arm/reducers/gripper | ToolBox Robotics | Official links and recorded package notices in hardware/SOURCES.md; no CAD/STL/3MF included |
| ROS 2, RoboStack, Pixi, NumPy, Pillow, optional speech/LLM tools | Respective upstream authors | External dependencies, installed separately; not relicensed by this repository |

Cyberbotics explains that some assets have separate notices: https://github.com/cyberbotics/webots/blob/R2025a/docs/guide/webots-license-agreement.md. Review the actual upstream asset notices when retrieving them. The GPL for DOC source is not a blanket license for downloaded assets or model weights.

The previous SixTetarm-RoboticArm release used a MIT software license and an inconsistent README description. Its historical software license is retained in LICENSES/Previous-MIT.txt for that release. The new release does not revoke permissions already granted for previously distributed copies. It does not purport to apply the historical MIT license to ToolBox Robotics designs.

Historical commits may contain earlier third-party model snapshots. Retaining Git history does not place those historical assets under the GPL of this release; their original rights and notices remain separate. The current source tree links to the official mechanical downloads.
