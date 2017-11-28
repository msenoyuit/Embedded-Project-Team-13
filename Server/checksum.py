def calculateChecksum(string):
    checksum = 0
    for char in string:
        checksum = (checksum + ord(char)) % 256

    return checksum
