import os
import rospy
import rospkg
from collections import defaultdict

from editvalues import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QFormLayout, QLabel, QLineEdit
from python_qt_binding.QtWidgets import *

class BasicInterface(Plugin):

    def __init__(self, context):
        super(BasicInterface, self).__init__(context)
        self.setObjectName('BasicInterface')
        # self.setFixedSize(300, 400)

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
            
        # Save URDF file
        # geturdf()

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('model_maker_helper'), 'resource', 'BasicInterface.ui')
        loadUi(ui_file, self._widget)
        self._widget.setFixedSize(600, 450)
        self._widget.setObjectName('BasicInterfaceUi')
        
        # Initialize dictionary
        # self.dict = defaultdict(int)
        self.labeldict = {}
        self.valdict = {}
        self.count = 1 # TO BE CHANGED LATER!!!!!!
        # self.initializedict()
                
        # Definitions
        self.createvarbtn = self._widget.findChild(QPushButton, 'addnewvariable')
        self.createvarbtn.clicked.connect(self.createlabel)
        
        self.newvarname = self._widget.findChild(QLineEdit, 'newvarname')
        
        self.form = self._widget.findChild(QFormLayout, 'formLayout')
        
        self.updatevalbutton = self._widget.findChild(QPushButton, 'updatevalues')
        self.updatevalbutton.clicked.connect(self.updatevalues)
        
        self.getyamlbtn = self._widget.findChild(QPushButton, 'getyamlbtn')
        self.getyamlbtn.clicked.connect(self.getyamlpath)
        
        # Show the widget
        context.add_widget(self._widget)

    def createlabel(self):
        name = self.newvarname.text()
        names = name.rsplit('/')     
        self.labeldict[self.count] = name
        
        if len(names) is not 2:
            # self.createvarbtn.setFocus()
            self.newvarname.clear()
            self.newvarname.setPlaceholderText('Enter both parent and property!')
            return
        
        # Creating new widgets for the new row
        self.label = QLabel("{}. Adjust {:s} values of {:s}".format(self.count, names[1], names[0]))
        self.val = QLineEdit()
        
        self.valdict[self.count] = self.val
        self.count += 1
        
        self.form.addRow(self.label, self.val)
        
    def getyamlpath(self):
        fname = QFileDialog.getOpenFileName(self._widget, 'Open file', '/home/',"Yaml File (*.yaml)")
        print(fname)
    
    def updatevalues(self):
        editvalues(self.labeldict, self.valdict)
        
        print('[LOG] Values have been updated!')
    
urdf = ""    
        
def geturdf():
    urdf = rospy.get_param("/robot_description")
    try:
        f = open("temp.urdf", "w")
        f.write(urdf)
        print("[LOG] Temporary urdf file created!")
    except Exception as e:
        print(e)
    
    
    
    

            
            
    
    
        
















# import os
# import rospy
# import rospkg

# from qt_gui.plugin import Plugin
# from python_qt_binding import loadUi
# from python_qt_binding.QtWidgets import QWidget, QPushButton

# class BasicInterface(Plugin):

#     def __init__(self, context):
#         super(BasicInterface, self).__init__(context)
#         self.setObjectName('BasicInterface')

#         # Process standalone plugin command-line arguments
#         from argparse import ArgumentParser
#         parser = ArgumentParser()
#         # Add argument(s) to the parser.
#         parser.add_argument("-q", "--quiet", action="store_true",
#                       dest="quiet",
#                       help="Put plugin in silent mode")
#         args, unknowns = parser.parse_known_args(context.argv())
#         if not args.quiet:
#             print ('arguments: ', args)
#             print ('unknowns: ', unknowns)

#         # Get path to UI file which should be in the "resource" folder of this package
#         ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'BasicInterface.ui')
#         # Extend the widget with all attributes and children from UI file
#         self._widget = QWidget()
#         loadUi(ui_file, self._widget)
#         # self.button = self._widget.findChild(QPushButton, "pushButton")
#         context.add_widget(self._widget)
#         # context.add_widget(self._widget)
        
