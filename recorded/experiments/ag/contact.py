"""Classify contacts with the CityPark road mesh without hiding impacts."""


def is_ground_contact(event):
    normal_z = float(event.get('normal', {}).get('z', 0))
    if normal_z < -0.75:
        return True
    return (event.get('object_name', '').startswith('MergedRoad')
            and float(event.get('penetration_depth', 1)) == 0
            and normal_z < -0.55)
