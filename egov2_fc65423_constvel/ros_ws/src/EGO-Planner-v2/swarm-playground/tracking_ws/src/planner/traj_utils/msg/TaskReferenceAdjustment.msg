# Team -> Cooperative 的任务级 phi0 建议（阶段 B / 任务书第六节）。
# Team realized triple 被接受后才发布；Cooperative manager 仍通过自己已有的
# rate-limited transition + transition validity 决定是否接受，Team 不是任务几何
# authority，只是建议者。
Header header
uint64 reference_generation   # 建议所基于的 CooperativeTaskReference.generation
uint64 team_transaction_id    # 产生该建议的 execution transaction
float64 recommended_phi0      # 由 realized bearings 与 slot 映射反解的公共相位
string reason
