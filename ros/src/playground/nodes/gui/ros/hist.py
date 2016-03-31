from PlotWindow import PlotWindow

import rospy
import sys, random
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import numpy
from std_msgs.msg import Int8

class OnlineHist(PlotWindow):
  def __init__(self):
    PlotWindow.__init__(self)

    self.window_size=20
    self.values=numpy.zeros((self.window_size))
    self.index=0

    rospy.init_node('visualizer', anonymous=True)
    self.subscriber = rospy.Subscriber("random_value", Int8, self.plotResults )


  def plotResults(self, data):
    print data
    self.axes.clear()
    self.axes.set_autoscaley_on(False)

    if self.index==self.window_size-1:
      self.index=0
    else:
      self.index=self.index+1
    self.values[self.index]=data.data 

    n, bins, patches = self.axes.hist(self.values, 10, (0, 10), normed=True, facecolor='green', alpha=0.75)

    output= "Data index "+str(self.index)
    min_x, max_x=self.axes.get_xlim()
    min_y, max_y=self.axes.get_ylim()   
    self.axes.text(max_x*0.6,max_y*0.7,output,horizontalalignment='left',verticalalignment='center')

    self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OnlineHist()
    window.show()
    app.exec_()