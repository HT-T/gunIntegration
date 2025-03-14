import sys
sys.path.append('..')
import csv
import os
import queue
import threading
from beetle import GunBeetle
from internal_utils import BEETLE_MAC_ADDR, bcolors
from collections import namedtuple

# Define GunUpdatePacket structure for sending ammo updates
GunUpdatePacket = namedtuple('GunUpdatePacket', ['player_id', 'bullets'])

# Constants
GUN_BEETLE = BEETLE_MAC_ADDR.P1_GUN.value
PLAYER_ID = 1  # Matching PLAYER_ID from gun.hpp

def dump_gun_data_to_csv(gun_queue):
    filename = input("Enter the filename to dump gun data to: ")
    target_file_path = f"gun_data/{filename}.csv"
    os.makedirs(os.path.dirname(target_file_path), exist_ok=True)
    
    with open(target_file_path, 'w') as output_csv:
        csv_writer = csv.writer(output_csv)
        csv_writer.writerow(["Player ID", "isFired", "Label"])
        for gun_data in gun_queue.queue:
            csv_writer.writerow([gun_data.playerID, gun_data.isFired, filename])

def print_gun_queue(gun_queue):
    """Function to print the contents of the gun collector queue"""
    if gun_queue.empty():
        print("Queue is empty")
        return
    
    print("\nCurrent Gun Collector Queue Contents:")
    print("-------------------------------------")
    print("Player ID | isFired")
    print("-------------------------------------")
    
    # We need to iterate through queue.queue without removing items
    for idx, item in enumerate(gun_queue.queue):
        print(f"{item.playerID:9} | {item.isFired}")
    
    print(f"\nTotal entries: {gun_queue.qsize()}")

def command_listener(incoming_queue, gun_collector_queue):
    """Thread function to listen for user commands and send ammo updates"""
    print("Command interface ready.")
    print("Enter 'ammo X' to set bullet count (where X is 0-6)")
    print("Enter 'print' to view the current gun collector queue contents")
    print("Enter 'quit' to exit")
    
    while True:
        command = input("> ")
        
        if command.lower() == 'quit':
            break
            
        elif command.lower() == 'print':
            print_gun_queue(gun_collector_queue)
            
        elif command.lower().startswith('ammo '):
            try:
                bullets = int(command.split()[1])
                if 0 <= bullets <= 6:  # GUN_MAGAZINE_SIZE is 6
                    print(f"Sending ammo update: {bullets} bullets")
                    # Create and send GunUpdatePacket
                    update_packet = GunUpdatePacket(player_id=PLAYER_ID, bullets=bullets)
                    incoming_queue.put(update_packet)
                else:
                    print("Bullet count must be between 0 and 6")
            except (IndexError, ValueError):
                print("Invalid command format. Use 'ammo X' where X is a number.")

if __name__=="__main__":
    gun_beetle = None
    color = bcolors.BRIGHT_CYAN
    incoming_queue = queue.Queue()  # Renamed from dummy_incoming_queue since we're actually using it
    gun_collector_queue = queue.Queue()
    
    try:
        # Start the gun beetle
        gun_beetle = GunBeetle(GUN_BEETLE, gun_collector_queue, incoming_queue, color)
        gun_beetle.start()
        
        # Start command listener thread
        cmd_thread = threading.Thread(target=command_listener, args=(incoming_queue, gun_collector_queue), daemon=True)
        cmd_thread.start()
        
        # Main thread waits for interrupt
        print("Gun test running. Use the command interface to send ammo updates.")
        print("Press Ctrl+C to exit and dump data to CSV.")
        
        while True:
            pass
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        if gun_beetle:
            gun_beetle.quit()
            gun_beetle.join()
        dump_gun_data_to_csv(gun_collector_queue)
        sys.exit(0)