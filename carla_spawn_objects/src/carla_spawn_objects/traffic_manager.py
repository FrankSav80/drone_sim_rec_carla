#!/usr/bin/env python

import carla
import rospy
from time import sleep
import random  # aggiunto per il seed

class CarlaTrafficManager:
    def __init__(self, seed_value=42):
        # Connessione al server di Carla
        self.client = carla.Client('carla-container.local', 2000)  # Cambia 'localhost' con l'IP di Carla se necessario
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        rospy.loginfo("Connessione al mondo di CARLA stabilita")

        # Inizializzazione del Traffic Manager
        self.traffic_manager = self.client.get_trafficmanager(9000)  # Assicurati che questa sia la porta corretta

        # Imposta il seed per garantire la ripetibilità
        self.traffic_manager.set_random_device_seed(seed_value)
        random.seed(seed_value)

        self.traffic_manager.set_global_distance_to_leading_vehicle(2.0)  # Distanza di sicurezza globale

    def manage_existing_vehicles(self):
        # Ottiene tutti gli attori presenti nel mondo
        actors = self.world.get_actors()
        print(f"Numero totale di attori nel mondo: {len(actors)}")

        # Filtra solo i veicoli
        vehicles = actors.filter('vehicle.*')

        # Collega ogni veicolo al Traffic Manager
        for vehicle in vehicles:
            vehicle.set_autopilot(True, self.traffic_manager.get_port())  # Attiva l'autopilot tramite Traffic Manager

            # Configurazione del Traffic Manager per ogni veicolo
            self.traffic_manager.vehicle_percentage_speed_difference(vehicle, -20)  # Riduce la velocità del 20%
            self.traffic_manager.ignore_lights_percentage(vehicle, 0)  # Rispetta i semafori
            self.traffic_manager.auto_lane_change(vehicle, True)  # Abilita cambio di corsia automatico

        rospy.loginfo(f"Gestiti {len(vehicles)} veicoli tramite il Traffic Manager.")

    def manage_existing_pedestrians(self):
        # Ottiene tutti gli attori presenti nel mondo
        actors = self.world.get_actors()

        # Filtra solo i pedoni e i loro controller
        pedestrians = actors.filter('walker.pedestrian.*')

        # Spawn dei controller per ogni pedone
        for pedestrian in pedestrians:

            # Spawn del controller per il pedone
            controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
            controller = self.world.spawn_actor(controller_bp, carla.Transform(), attach_to=pedestrian)

            controller.start()
            target_location = self.world.get_random_location_from_navigation()
            if target_location:  # Assicurati che la posizione di destinazione sia valida
                controller.go_to_location(target_location)
                controller.set_max_speed(1.5)  # Velocità massima del pedone
            else:
                 rospy.logwarn("Impossibile ottenere una posizione di destinazione valida per il pedone.")

        rospy.loginfo(f"Gestiti {len(pedestrians)} pedoni tramite il Traffic Manager.")

    def update_simulation(self, event):
        # Aggiorna la simulazione
        self.world.tick()  # Esegui il tick del mondo per aggiornare la simulazione

if __name__ == "__main__":
    rospy.init_node('carla_traffic_manager_node')

    manager = CarlaTrafficManager()

    # Attendere per assicurarsi che tutto sia spawnato in Carla
    rospy.loginfo("Attesa di 10 secondi per assicurarsi che tutti gli attori siano spawnati.")
    sleep(10)  # Aspetta un po' per permettere lo spawn di veicoli e pedoni

    # Gestione del Traffic Manager per veicoli e pedoni esistenti
    manager.manage_existing_vehicles()
    manager.manage_existing_pedestrians()

    rospy.loginfo("Gestione del Traffic Manager completata.")

    rospy.spin()
