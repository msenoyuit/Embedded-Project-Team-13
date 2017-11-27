# Rover IDs
roverIDs = {
    "ROVER_0": '0',
    "ROVER_1": '1'
}

# Pi Commands
commands = {
    "MOVE_COMMAND":     '0', # direction
    "READ_COMMAND":     '1', # direction
    "PICKUP_COMMAND":   '2', # direction
    "STREAM_START":     '3', # sensor
    "STREAM_STOP":      '4', # sensor
    "RELEASE_COMMAND":  '5'
}

# Pi Specifiers
specifiers = {
    "NORTH_MOVE":       '0',
    "EAST_MOVE":        '1',
    "SOUTH_MOVE":       '2',
    "WEST_MOVE":        '3',
    "LINE_SENSOR":      '4',
    "COLOR_SENSOR":     '5',
    "DISTANCE_SENSOR":  '6',
    "":                 ''
}

# Pi Flags
flags = {
    "COMMAND_RECEIVED": '0',
    "COMMAND_FINISHED": '1',
    "EVENT_ALERT":      '2'
}

'''
Send move command
Rover Acks
If block is in the way
    Rover sends event alert, and doesn't change location
    Server can send a read command
        Rover acks
        Rover will go forwards, read, send a single sensor reading message
        Rover will go back to current position
        Rover will send command complete
If block isn't in the way
    Rover does move and sends command complete when at next intersect
'''