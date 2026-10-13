# R7.2 perception on simulator scans (nav_maze)

Revision `07a2cd0`, molar1 aarch64; 3 bags, 17 scans each (every 40 after the first 20).

Boundaries: adjacent valid reference beams > 0.20 m apart, matched within 2 beams.

| Method | Precision | Recall | F1 | Clusters / scan | True segments / scan | ms p50 |
|---|---|---|---|---|---|---|
| DBSCAN | 0.659 | 0.147 | 0.240 | 6.1 | 20.3 | 2.65 |
| Euclidean clustering | 0.659 | 0.147 | 0.240 | 6.1 | 20.3 | 2.28 |
| Obstacle detector (single linkage) | 0.851 | 0.543 | 0.663 | 12.4 | 20.3 | 2.15 |
| Scan clusterer, adaptive breakpoint | 0.632 | 0.213 | 0.319 | 6.8 | 20.3 | 0.59 |
| Scan clusterer, fixed breakpoint | 0.796 | 0.544 | 0.646 | 13.4 | 20.3 | 0.33 |
