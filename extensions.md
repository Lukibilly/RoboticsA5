# Extensions

## 1.Nearest Neighbor
- Node exhaustion: Count connect successes and fails when connecting. Exhaust if fails > 100 and not use as a nearest neighbor.

## 2.Sampling
- Goal Bias: Doesn't work for bi-directional trees, they already try to connect to each other, i.e., connecting goal and start. 
- Gaussian Sampling: Needs testing.
- Bridge Sampling: Needs testing. Should be really good. 
- Tracking nearest node and distance from tree a to tree b and the other way around. Use this information for sampling strategy.

## 3.Distance metric
- Distance metric in workspace??? (Jacobian Transpose)

## 4. Connect/Extend

## 5. Solve algorithm