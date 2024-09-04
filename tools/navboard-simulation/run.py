import logging
import time

from RoveComm_Python.rovecomm import *

# Get RoveComm Manifest
manifest = get_manifest()


def handle_core_commands(packet):
    """
    Used to handle incoming core commands and create the next point appropriately
    """

    if packet is None:
        raise TypeError

    if packet.data_id == this.manifest["Core"]["Commands"]["DriveLeftRight"]["dataId"]:
        # Do something here
        print("DriveLeftRight Packet Received")


def handle_autonomy_commands(packet):
    """
    Used to handle incoming autonomy commands and help create the next point appropriately
    """

    if packet is None:
        raise TypeError

    if (
        packet.data_id
        == this.manifest["Autonomy"]["Commands"]["StartAutonomy"]["dataId"]
    ):
        # Do something here
        print("StartAutonomy Packet Received")
    elif (
        packet.data_id
        == this.manifest["Autonomy"]["Commands"]["DisableAutonomy"]["dataId"]
    ):
        # Do something here
        print("DisableAutonomy Packet Received")
    elif (
        packet.data_id
        == this.manifest["Autonomy"]["Commands"]["AddPositionLeg"]["dataId"]
    ):
        # Do something here
        print("AddPositionLeg Packet Received")
    elif (
        packet.data_id
        == this.manifest["Autonomy"]["Commands"]["AddMarkerLeg"]["dataId"]
    ):
        # Do something here
        print("AddMarkerLeg Packet Received")
    elif (
        packet.data_id == this.manifest["Autonomy"]["Commands"]["AddGateLeg"]["dataId"]
    ):
        # Do something here
        print("AddGateLeg Packet Received")
    elif (
        packet.data_id
        == this.manifest["Autonomy"]["Commands"]["ClearWaypoints"]["dataId"]
    ):
        # Do something here
        print("ClearWaypoints Packet Received")
    elif (
        packet.data_id == this.manifest["Autonomy"]["Commands"]["SetMaxSpeed"]["dataId"]
    ):
        # Do something here
        print("SetMaxSpeed Packet Received")


def handle_autonomy_telemetry(packet):
    """
    Used to handle incoming autonomy telemetry and help create the next point appropriately
    """

    if packet is None:
        raise TypeError

    if (
        packet.data_id
        == this.manifest["Autonomy"]["Telemetry"]["CurrentState"]["dataId"]
    ):
        # Do something here
        print("CurrentState Packet Received")
    elif (
        packet.data_id
        == this.manifest["Autonomy"]["Telemetry"]["ReachedMarker"]["dataId"]
    ):
        # Do something here
        print("CurrentState Packet Received")


# Initialize RoveComm Nodes
# Nav Board: This node will only be responsible to sending packets with GPS and IMU
nav_node = RoveComm(11000, ("", manifest["Nav"]["Port"]))
# Core Board: Subscribe to Core Board to calculate the current location
core_node = RoveComm(11000, ("", manifest["Core"]["Port"]))
# Autonomy: Subscribe to Autonomy to know where we are headed to create starting point
autonomy_node = RoveComm(11000, ("", manifest["Autonomy"]["Port"]))

# Subscribe to Core and Autonomy
core_node.udp_node.subscribe(core.manifest["Core"]["Ip"])
autonomy_node.udp_node.subscribe(core.manifest["Core"]["Ip"])

# Set Up Callbacks
core_node.set_callback(
    manifest["Core"]["Commands"]["DriveLeftRight"]["dataId"], handle_core_commands
)
autonomy_node.set_callback(
    manifest["Autonomy"]["Commands"]["StartAutonomy"]["dataId"],
    handle_autonomy_commands,
)
autonomy_node.set_callback(
    manifest["Autonomy"]["Commands"]["DisableAutonomy"]["dataId"],
    handle_autonomy_commands,
)
autonomy_node.set_callback(
    manifest["Autonomy"]["Commands"]["AddPositionLeg"]["dataId"],
    handle_autonomy_commands,
)
autonomy_node.set_callback(
    manifest["Autonomy"]["Commands"]["AddMarkerLeg"]["dataId"], handle_autonomy_commands
)
autonomy_node.set_callback(
    manifest["Autonomy"]["Commands"]["AddGateLeg"]["dataId"], handle_autonomy_commands
)
autonomy_node.set_callback(
    manifest["Autonomy"]["Commands"]["ClearWaypoints"]["dataId"],
    handle_autonomy_commands,
)
autonomy_node.set_callback(
    manifest["Autonomy"]["Commands"]["SetMaxSpeed"]["dataId"], handle_autonomy_commands
)
autonomy_node.set_callback(
    manifest["Autonomy"]["Telemetry"]["CurrentState"]["dataId"],
    handle_autonomy_telemetry,
)
autonomy_node.set_callback(
    manifest["Autonomy"]["Telemetry"]["ReachedMarker"]["dataId"],
    handle_autonomy_telemetry,
)

LatLon = [0.0, 0.0]
PitchYawRole = [0.0, 0.0, 0.0]
Bearing = 0

while True:
    # Send RoveComm Nav Packets
    packet = RoveCommPacket(
        manifest["Nav"]["Telemetry"]["GPSLatLon"]["dataId"], "f", (LatLon[0], LatLon[1])
    )
    nav_node.write(packet, False)

    packet = RoveCommPacket(
        manifest["Nav"]["Telemetry"]["IMUData"]["dataId"],
        "f",
        (PitchYawRole[0], PitchYawRole[1], PitchYawRole[2]),
    )
    nav_node.write(packet, False)

    packet = RoveCommPacket(
        manifest["Nav"]["Telemetry"]["CompassData"]["dataId"], "f", (Bearing,)
    )
    nav_node.write(packet, False)
