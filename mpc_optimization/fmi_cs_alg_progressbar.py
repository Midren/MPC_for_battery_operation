from timeit import default_timer as timer

import numpy as np
from tqdm import tqdm
from pyfmi import fmi
from pyfmi.fmi_algorithm_drivers import FMICSAlg


class FMICSAlgWithProgressBar(FMICSAlg):
    def solve(self):
        """
        Runs the simulation.
        """
        result_handler = self.result_handler
        h = (self.final_time - self.start_time) / self.ncp
        grid = np.linspace(self.start_time, self.final_time, self.ncp + 1)[:-1]

        status = 0
        final_time = self.start_time

        # For result writing
        start_time_point = timer()
        result_handler.integration_point()
        self.timings["storing_result"] = timer() - start_time_point

        # Start of simulation, start the clock
        time_start = timer()

        for time_step in tqdm(grid):
            status = self.model.do_step(time_step, h)
            self.status = status

            if status != 0:

                if status == fmi.FMI_ERROR:
                    result_handler.simulation_end()
                    raise fmi.FMUException(
                        "The simulation failed. See the log for more information. Return flag %d." %
                        status)

                elif status == fmi.FMI_DISCARD and (isinstance(self.model, fmi.FMUModelCS1)
                                                    or isinstance(self.model, fmi.FMUModelCS2)):

                    try:
                        if isinstance(self.model, fmi.FMUModelCS1):
                            last_time = self.model.get_real_status(fmi.FMI1_LAST_SUCCESSFUL_TIME)
                        else:
                            last_time = self.model.get_real_status(fmi.FMI2_LAST_SUCCESSFUL_TIME)
                        if last_time > time_step:  # Solver succeeded in taken a step a little further than the last time
                            self.model.time = last_time
                            final_time = last_time

                            start_time_point = timer()
                            result_handler.integration_point()
                            self.timings["storing_result"] += timer() - start_time_point
                    except fmi.FMUException:
                        pass
                break
                # result_handler.simulation_end()
                # raise Exception("The simulation failed. See the log for more information. Return flag %d"%status)

            final_time = time_step + h

            start_time_point = timer()
            result_handler.integration_point()
            self.timings["storing_result"] += timer() - start_time_point

            if self.options["time_limit"] and (timer() - time_start) > self.options["time_limit"]:
                raise fmi.TimeLimitExceeded(
                    "The time limit was exceeded at integration time %.8E." % final_time)

            if self.input_traj is not None:
                self.model.set(self.input_traj[0], self.input_traj[1].eval(time_step + h)[0, :])

        # End of simulation, stop the clock
        time_stop = timer()

        result_handler.simulation_end()

        if self.status != 0:
            if not self.options["silent_mode"]:
                print(
                    'Simulation terminated prematurely. See the log for possibly more information. Return flag %d.'
                    % status)

        # Log elapsed time
        if not self.options["silent_mode"]:
            print('Simulation interval    : ' + str(self.start_time) + ' - ' + str(final_time) +
                  ' seconds.')
            print('Elapsed simulation time: ' + str(time_stop - time_start) + ' seconds.')

        self.timings["computing_solution"] = time_stop - time_start - self.timings["storing_result"]
