# Validation scope

`offline_tests.json` records 62 passing offline tests. Existing reconstruction
site-packages were appended to the active Python environment; no dependency
installation or original simulator modifications were performed.
`native_profiles.json` records clean-base patch application and exact overlay
hashes for both profiles; a complete moving-deck source assembly also passed.
`source_preservation.json` verifies every copied original source was unchanged.
`cli_entry_points.json` records --help for all four task wrappers without connecting.
`validation.json` combines final checks. No UE/plugin compilation or four live
experiments were rerun while packaging. `evidence/native_historical` test reports
are explicitly older results, not new C++ test execution.
