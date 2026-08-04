# System Requirements

Status: outline — not yet finalized.

Derived from `project_plan.md` → "Engineering requirements". To be written up as testable, numbered requirements before implementation of each phase begins.

## Functional requirements
- Detect one cooperative small drone within an initial test envelope of 3–10 meters.
- Update detections at a defined minimum rate.
- Maintain a track through brief missed detections.
- Estimate position and velocity with documented error.
- Record synchronized raw and processed data.
- Replay recorded datasets without hardware.
- Run the tracking pipeline from one launch command.

## Verification requirements
- Automated tests for coordinate conversion, filtering, association, and state estimation.

## Non-goals
- No interference, jamming, takeover, or interception functions (see "Legal and flight boundary" in the project plan).
- No maximum-range claims until data is collected.
