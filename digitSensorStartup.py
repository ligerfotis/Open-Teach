import hydra
from openteach.components.initializers import DigitSensors

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'digitSensor')
def main(configs):
    digitSensors =DigitSensors(configs)
    processes = digitSensors.get_processes()

    for process in processes:
        process.start()

    for process in processes:
        process.join()

if __name__ == '__main__':
    main()