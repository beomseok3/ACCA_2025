from pyproj import Proj, transform

# WGS84(EPSG:4326) to EPSG:2097
source_epsg = 'EPSG:4326'  # UTM Zone 32N
target_epsg = 'EPSG:2097'  # WGS84 EPSG code

lon = 126.957132 #lon
lat = 37.496136  #lat

source_proj = Proj(init=source_epsg)
target_proj = Proj(init=target_epsg)

x, y = transform(source_proj, target_proj, lon, lat)

print(f"tm Coordinates (x, y): {x}, {y}")


# EPSG:2097 to WGS84(EPSG:4326)

# source_epsg = 'EPSG:2097'  # WGS84 EPSG code
# target_epsg = 'EPSG:4326'  # UTM Zone 32N

# x = 19432.021484375
# y = 51963.4140625

# source_proj = Proj(init=source_epsg)
# target_proj = Proj(init=target_epsg)

# lon, lat = transform(source_epsg, target_epsg, x, y)

# print(f"WGS84 Coordinates (lon, lat): {lon}, {lat}")