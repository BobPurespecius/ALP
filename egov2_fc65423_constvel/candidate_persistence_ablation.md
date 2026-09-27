# Candidate Persistence Ablation

## Setup

- Stage 5 control: persistence=false (original Stage 5 Candidate Hysteresis).
- Persistence: persistence=true (experimental Candidate Persistence build).
- Same Mode C scene and parameters, `candidate_switch_margin=0.12`.
- RViz disabled; runtime 85 s.

## Results

| Metric | Stage 5 | Persistence |
|---|---:|---:|
| triggered events | 141 | 126 |
| history opportunities | 20 | 25 |
| history attempted | 18 | 25 |
| history side missing | 2/20 | 0/25 |
| history valid | N/A | 11 |
| history retained | N/A | 11 |
| history replaced/invalid | N/A | 14 |
| PLUS->MINUS | 2 | 1 |
| MINUS->PLUS | 1 | 1 |
| blocked switches | 0 | 0 |
| avg side optimizations / trigger | 1.929 | 1.944 |

## Conclusion

Candidate Persistence reduced history-side missing from 10% to 0%, but same-obstacle
switches only changed from 3 to 2. The sample is too small to establish a significant
stability improvement. Most history-side failures were optimization failures or failure
to meet the existing clearance-improvement threshold, rather than candidate absence.

Therefore Candidate Persistence is not part of the final algorithm. Stage 5 hysteresis
is retained. The next direction is Temporal Retiming rather than additional spatial
persistence logic.

## Preserved Artifacts

- Stage 5 control log: `candidate_persistence_control.log`
- Persistence log: `candidate_persistence_enabled_final.log`
- Stage 5 control visibility and trajectory CSVs: `candidate_persistence_control_visibility*.csv`
- Persistence visibility and trajectory CSVs: `candidate_persistence_enabled_final_visibility*.csv`
