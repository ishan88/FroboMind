import rospy

class C():
    def __init__(self):
        self.FloatStamped = np.dtype(np.float64)
        self.FloatArrayStamped = np.dtype([(np.float64)])
        self.implement = FloatStamped()
        print(" Data Type is  "+str(self.implement))
        
        
        
        
if __name__ == '__main':
    C()