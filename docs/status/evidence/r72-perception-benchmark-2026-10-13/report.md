# R7.2 perception benchmark (synthetic scenes)

Revision `42d1ccb`, molar1 aarch64; 100 laser scenes, 20 point clouds.

## Laser obstacle detection

| Method | Precision | Recall | Median centroid offset (m) | ms p50 / max |
|---|---|---|---|---|
| obstacle_detector | 0.794 | 0.998 | -0.0305 | 1.16 / 1.36 |
| scan_clusterer_fixed | 0.766 | 0.996 | -0.0303 | 0.79 / 0.83 |
| scan_clusterer_adaptive | 0.862 | 0.989 | -0.0305 | 1.12 / 1.17 |
| dbscan | 0.794 | 0.998 | -0.0305 | 1.40 / 1.57 |
| euclidean | 0.794 | 0.998 | -0.0305 | 1.20 / 1.38 |

## Ground removal (per point)

| Method | Precision | Recall | ms p50 |
|---|---|---|---|
| pointcloud_segmenter (height 0.1 m) | 0.947 | 0.429 | 1.1 |
| ransac_ground_removal | 0.929 | 1.000 | 38.3 |

## Object clustering (non-ground points)

| Method | Boxes found | Clusters per scene | ms p50 |
|---|---|---|---|
| euclidean_clusterer | 80/80 | 11.6 | 15.8 |
| dbscan_clusterer | 80/80 | 15.6 | 15.8 |
