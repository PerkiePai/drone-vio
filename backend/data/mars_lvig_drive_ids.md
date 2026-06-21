# MARS-LVIG dataset — Google Drive file IDs

Source folder (public): https://drive.google.com/drive/folders/1aG21le4QZl9LhSLE1O0vuVDQ8YgAwdas
Dataset page: https://mars.hku.hk/dataset.html

ROS1 `.bag` files. For pure visual-inertial odometry (OpenVINS) we only need the
**camera + IMU** topics; LiDAR/GNSS in the bag are ignored at runtime (GNSS/RTK
kept only as an offline ground-truth yardstick).

> NOTE: anonymous CLI downloads (gdown/curl) are currently blocked by Google's
> per-file quota: "Too many users have viewed or downloaded this file recently."
> Use a logged-in browser session or authenticated rclone/gdown instead.

Authoritative IDs (read from the live folder, 2026-06-21):

| Sequence              | File ID                              |
|-----------------------|--------------------------------------|
| AMtown01.bag          | 1mXEWG8sxR0V0JHaSD-upAgK21Ge6ybNa    |
| AMtown02.bag          | 15bio33_AMrumJM-zZCzvdo9P1JBirhcQ    |
| AMtown03.bag          | 1m6-Um5Jd3mLRKHpBlKgGbtoV-kPv4eKa    |
| AMvalley01.bag        | 1NTecR3tb2-NYZDPH_p94bFy3lYmsQ53b    |
| AMvalley02.bag        | 1g5YQ6Po0gF_HhcJ24A03AJOOrybmSjOJ    |
| AMvalley03.bag        | 18B2yu7HHPHd-Wy3HHgAC74_5FsgBK7F3    |
| HKairport01.bag       | 1dj_7Htdb1rtRsiQ-HqcndrBxV4GOr_2H    |
| HKairport02.bag       | 1hxSdFyULhS2MlVG5Aba9YWFc9DambsuA    |
| HKairport03.bag       | 1V2Z8eUuVOanpdZiga7LcskgzfJD3gEqn    |
| HKairport_GNSS01.bag  | 1F61oziWPbeaqP5fFtuiqvl_3BvrvWwz0    |
| HKairport_GNSS02.bag  | 1OVFhhpkFReSy7j1AYIYQG4rbWL_ZRP-3    |
| HKairport_GNSS03.bag  | 1ORuC3xyi7yonIzD8mTmKDWRU0T6snu9n    |
| HKisland01.bag        | 1nhX7hGyjCaoIfqc2b3PhAaHHu2Vv8wOQ    |
| HKisland02.bag        | 1CPOE1y6fVz20f4tbFv44TWMatihb8JTZ    |
| HKisland03.bag        | 1Now3Dz8UIHkwya8YvoJYHzukEWEBAobB    |
| HKisland_GNSS01.bag   | 1YG22xjU6q-AECsizsrshgfgq0lUWDe7j    |
| HKisland_GNSS02.bag   | 1UDcZgElsNepsJXgE9Wx1djHrLNiNwqyP    |
| HKisland_GNSS03.bag   | 136xN27vdpUe88NugHhPKArZBveiYD3hj    |

(Featureless_GNSS01-03 also exist in the folder; IDs not yet captured.)

Target sequence for VIO bring-up: **AMvalley01.bag** (forest/valley, nadir).
Browser link: https://drive.google.com/file/d/1NTecR3tb2-NYZDPH_p94bFy3lYmsQ53b/view

Download destination (gitignored): `_in/mars-lvig/AMvalley01.bag`
