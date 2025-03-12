import hydra
from openteach.components.initializers import GelSightSensors

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'gelsightSensor')
def main(configs):
    gelsightSensors =GelSightSensors(configs)
    processes = gelsightSensors.get_processes()

    for process in processes:
        process.start()

    for process in processes:
        process.join()

if __name__ == '__main__':
    main()