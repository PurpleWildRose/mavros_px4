message("               Mavlink libraries               ")
message("===============================================")

set(MAVLINK_FOUND 1)

set(MAVLINK_DEFINITIONS "")
set(MAVLINK_INCLUDE_DIR "${CAMKE_CURRENT_LIST_DIR}"/../../include/mavlink)
set(MAVLINK_INCLUDE_DIRS "${CAMKE_CURRENT_LIST_DIR}"/../../include/mavlink)

set(MAVLINK_VERSION_STR "V2")

add_definitions(${MAVLINK_DEFINITIONS})
include_directories(${MAVLINK_INCLUDE_DIR})
