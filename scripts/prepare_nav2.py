"""Generate only Nav2 configuration; never overwrite SLAM/EKF settings."""
import json
from pathlib import Path
import yaml
from r1pro_waypoint_nav.nav2_config import navigation_profile,configure_nav2

root=Path(__file__).resolve().parents[1]
geometry=json.loads((root/'reports/navigation_geometry.json').read_text())
profile=navigation_profile(geometry)
defaults=yaml.safe_load(Path('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml').read_text())
config=root/'src/r1pro_waypoint_nav/config'
(config/'navigation_profile.json').write_text(json.dumps(profile,indent=2))
(config/'nav2.yaml').write_text(yaml.safe_dump(configure_nav2(defaults,profile),sort_keys=False))
print(json.dumps(profile,indent=2))
