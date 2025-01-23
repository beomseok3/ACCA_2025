import numpy as np

class PI_period_checker:
    def __init__(self):
        self.alpha = 0.1
        self.period = 0.0
        self.heat = 0.0
        self.heat_time = []
        self.count = 0
        
    def measure_critical_oscillation_period(self, p_gain, i_gain, desired_value):
        """
        Measure the oscillation period of the system.

        Returns:
            float: Oscillation period.
        """
        # Measure the time it takes for the system to oscillate
        # (e.g., the time between two consecutive peaks or valleys in the response)
        
        with open("/home/ps/pid_test/p:{}, i:{}, v:{}, battery:48.8.txt".format(p_gain, i_gain, desired_value)) as f:
            lines = f.readlines()
            tv = list(map(float, line.strip().split(", ")) for line in lines[:-1]) # [ [t0, v0], [t1, v1], ... ] except last line
            for i in range(1, len(tv) -1):
                tv[i][2] = self.is_peak(tv, i)
            for tv in tv[1:-1]:
                if tv[2] == True:
                    if self.heat == 0.:
                        self.heat = tv[0] # initialize
                        self.heat_time += list(tv[1])
                    else:
                        if self.same_peak(self.heat, tv[0]): 
                            self.count += 1 # count the number of peaks
                        else:
                            self.heat_time = []
                            self.heat = tv[0] # reinitialize
                            self.heat_time += list(tv[1])
                            self.count = 0
                    
                    if self.count == 50:
                        for i in range(0,len(self.heat_time)-1):
                            print(self.low_pass_filter(self.heat_time[i+1] - self.heat_time[i]))
                        break
                    
    def low_pass_filter(self, time):
        # low pass filter 
        self.period = self.alpha * self.period + (1 - self.alpha)*time
        return self.period
                        
                
    def is_peak(self,tv, i):
        """
        Check if the given point is a peak.

        Parameters:
            tv (list): A list of time-value pairs.
            i (int): Index of the point to check.

        Returns:
            bool: True if the point is a peak, False otherwise.
        """
        if i == 0 or i == len(tv) - 1:
            return False

        return tv[i][1] > tv[i - 1][1] and tv[i][1] > tv[i + 1][1]
    
    def same_peak(self, a, b):
        return abs(a - b) < 0.1 # 0.1 is a threshold value
        

                 

def ziegler_nichols_pi_tuning(Ku, Tu):
            """
            Ziegler-Nichols PI Tuning Function.

            Parameters:
                Ku (float): Ultimate gain (maximum gain for sustained oscillations). # P gain이 낮으면 오버슛없이 수렴,, 높으면 에러 발산 그 사이에서 결정 (tunnig값은 0에서 1씩 올리면서 해보기)
                Tu (float): Oscillation period at ultimate gain.

            Returns:
                dict: A dictionary containing Kp (proportional gain) and Ki (integral gain).
            """
            # Ziegler-Nichols formulas for PI tuning
            Kp = 0.45 * Ku
            Ki = 0.54 * Ku / Tu    # Integral gain

            return {"Kp": Kp, "Ki": Ki}

def inverse_ziegler_nichols_pi_tuning(Kp, Ki):
    """
    Inverse Ziegler-Nichols PI Tuning Function.

    Parameters:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.

    Returns:
        dict: A dictionary containing Ku (ultimate gain) and Tu (oscillation period at ultimate gain).
    """
    # Inverse Ziegler-Nichols formulas for PI tuning
    Ku = 1.0 / 0.45 * Kp
    Tu = 0.54 * Ku / Ki

    return {"Ku": Ku, "Tu": Tu}


# Example usage
# Ku = 6.0  # Example ultimate gain
# Tu = 2.5  # Example oscillation period
# gains = ziegler_nichols_pi_tuning(Ku, Tu)
# print("Tuned PI Gains:", gains)

# Kp = 2.07
# Ki = 0.85
# exp = inverse_ziegler_nichols_pi_tuning(Kp, Ki)
# print("Expected Ku and Tu:", exp)

pid = PI_period_checker()
pid.measure_critical_oscillation_period(2.07, 0.85, 6.5)
