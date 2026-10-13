# R7.2 perception on simulator scans (nav_maze)

Revision `07a2cd0`, molar1 aarch64; 3 bags, 17 scans each (every 40 after the first 20).

Boundaries: adjacent valid reference beams > 0.20 m apart, matched within 2 beams.

| Method | Precision | Recall | F1 | Clusters / scan | True segments / scan | ms p50 |
|---|---|---|---|---|---|---|
| DBSCAN | 0.586 | 0.106 | 0.180 | 5.3 | 20.1 | 2.58 |
| Euclidean clustering | 0.586 | 0.106 | 0.180 | 5.3 | 20.1 | 2.17 |
| Obstacle detector (single linkage) | 0.862 | 0.553 | 0.674 | 12.3 | 20.1 | 2.08 |
| Scan clusterer, adaptive breakpoint | 0.702 | 0.194 | 0.304 | 6.3 | 20.1 | 0.57 |
| Scan clusterer, fixed breakpoint | 0.862 | 0.553 | 0.674 | 13.3 | 20.1 | 0.32 |
