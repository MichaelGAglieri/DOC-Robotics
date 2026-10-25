# Architecture

The ROS package `doc_lab` resolves explicit laboratory commands against a local JSON catalog and observed simulation state. A mission planner builds a preview, revalidates the scene before committing and records the outcome locally. The Webots controller owns simulated motion and returns correlated outcomes over loopback UDP.

```text
Optional speech → explicit command → ROS coordinator + local catalog
                                      │
                                      ├→ mission preview / scene revalidation / journal
                                      ├→ UDP → Webots controller → observed simulated state
                                      └→ optional Ollama conversation → text / local speech
```

`laboratorio.json`, the scene generator and the five-axis kinematics define the demonstration geometry. The world uses DocArm5 derived from Cyberbotics UR3e; this is separate from the physical EB-15-based DOC geometry.

Robot Console is a separate macOS app. It controls the Mega over a confirmed serial protocol and can observe ROS 2 topics. That observer does not turn the simulated coordinates into physical robot commands. Local conversations, pose sequences, limits and mission journals are runtime data and are not versioned.
