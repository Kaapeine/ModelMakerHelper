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
            print ('arguments: ', args)
            print ('unknowns: ', unknowns)
            
        # Save URDF file
        # geturdf()

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('model_maker_helper'), 'resource', 'BasicInterface.ui')
        loadUi(ui_file, self._widget)
        self._widget.setFixedWidth(600)
        self._widget.setObjectName('BasicInterfaceUi')
        
        # Initialize dictionaries and other things
        self.yamlpath = ""
        self.labeldict = {}
        self.valdict = {}
        self.textboxdict = {}
        self.count = 0 
                
        # Definitions
        self.createvarbtn = self._widget.findChild(QPushButton, 'addnewvariable')
        self.createvarbtn.clicked.connect(self.createlabel)
        
        self.newvarname = self._widget.findChild(QLineEdit, 'newvarname')
        
        self.form = self._widget.findChild(QFormLayout, 'formLayout')
        
        self.getyamlbtn = self._widget.findChild(QPushButton, 'getyamlbtn')
        self.getyamlbtn.clicked.connect(self.getyamlpath)
        
        self.updatevalbutton = self._widget.findChild(QPushButton, 'updatevalues')
        self.updatevalbutton.clicked.connect(self.updatevalues)
        
        #TODO 
        self.deletevalbutton = self._widget.findChild(QPushButton, 'deletevariable')
        self.deletevalbutton.clicked.connect(self.deletevalues)
        self.deletevalnum = self._widget.findChild(QLineEdit, 'deletevarnum')
        
        self.updaterobotbtn = self._widget.findChild(QPushButton, 'updaterobotbtn')
        self.updaterobotbtn.clicked.connect(self.updaterobot)
        
        self.selectxacrobtn = self._widget.findChild(QPushButton, 'selectxacrobtn')
        self.selectxacrobtn.clicked.connect(self.selectxacro)
        
        self.updategazebobtn = self._widget.findChild(QPushButton, 'updategazebobtn')
        self.updategazebobtn.clicked.connect(self.updategazebo)
        
        # Show the widget
        context.add_widget(self._widget)

    def createlabel(self):
        self.name = self.newvarname.text()
        self.names = self.name.rsplit('/')     
                
        if len(self.names) is not 2:
            self.newvarname.clear()
            self.newvarname.setPlaceholderText('Enter "parent/property"')
            return
    
        self.labeldict[self.count] = self.names[1]
        
        # Creating new widgets for the new row
        self.label = QLabel("{}. Adjust {:s} values of {:s}".format(self.count+1, self.names[1], self.names[0]))
        self.val = QLineEdit()
        self.textboxdict[self.count] = self.val
        
        #TODO set placeholder text for newly added textboxes
        # self.val.setPlaceholderText('0.0')
        # self.val.setAlignment(Qt.AlignCenter)
        
        self.count += 1
        # print(self.count)
        
        self.updatefieldsfromyaml()
        self.form.addRow(self.label, self.val)
        
    def getyamlpath(self):
        self.yamlpath = QFileDialog.getOpenFileName(self._widget, 'Open file', '/home/',"Yaml File (*.yaml)")
        self.updatefieldsfromyaml()
        if self.yamlpath != ('', ''):
            self.showaffirmation("Values added from the Yaml file!")
        
    def updatefieldsfromyaml(self):
        # print(self.labeldict)
        if self.yamlpath != ('', '') and self.yamlpath != '' and self.yamlpath is not None:
            self.yamlfile = openyamlfile(self.yamlpath)
            self.yamlfilekeys = self.yamlfile.keys()
            self.yamlfilevals = self.yamlfile.values()
            # print(self.yamlfile)
            # print(self.yamlfilevals)
            # print(self.textboxdict)
            for c in range(self.count):
                if self.labeldict[c] in self.yamlfilekeys:
                    self.obj = self.textboxdict[c]
                    pos = self.yamlfilekeys.index(self.labeldict[c])
                    self.obj.setText(str(self.yamlfilevals[pos]))
    
    def updatevalues(self):
        if self.yamlpath == "":
            self.showmessagebox("Please select a Yaml file!")
            return
        
        # Get values from text box objects
        for c in range(self.count):
            self.obj = self.textboxdict[c]
            self.valdict[c] = self.obj.text()
            
        # print(self.labeldict)
        # print(self.valdict)
        
        flag = True
        flag = editvalues(self.labeldict, self.valdict, self.yamlpath)
        if not flag:
            self.showmessagebox("A field has been left empty")
            flag = True
        else: 
            print('[LOG] Values have been updated!')
            self.showaffirmation('Values have been updated in the Yaml file')
    
    def showmessagebox(self, text):
        self.msg = QMessageBox()
        self.msg.setIcon(QMessageBox.Warning)
        self.msg.setWindowTitle('Warning!')
        self.msg.setText(text)
        self.msg.show()
            
    def showaffirmation(self, text):
        self.msg = QMessageBox()
        self.msg.setIcon(QMessageBox.Warning)
        self.msg.setWindowTitle('Done!')
        self.msg.setText(text)
        self.msg.show()
    
    def deletevalues(self):
        num = self.deletevalnum.text()
        #TODO Add delete values features
        
    def selectxacro(self):
        self.xacropath = QFileDialog.getOpenFileName(self._widget, 'Open file', '/home/',"Xacro File (*.xacro)")
        if self.xacropath != ('', ''):
            self.showaffirmation("Xacro file added!")
        
    def updaterobot(self):
        try:
            self.urdfpath = os.path.join(os.getcwd(), "tempurdfs", "temp.urdf")
            os.system("xacro --inorder {:s} > {:s}".format(self.xacropath[0], self.urdfpath))
            os.system("rosparam set /robot_description {:s}".format(self.urdfpath))
            
            os.system("roslaunch model_maker_helper temp.launch")
            self.showaffirmation("URDF updated! Tick and untick the robot description to refresh your model!")
        except:
            self.showmessagebox("Select your URDF file!")
            
    def updategazebo(self):
        try:
            os.system("rosservice call /gazebo/delete_model agv_v1")
            os.system("roslaunch model_maker_helper gazebo.launch")
            os.system("echo 'Gazebo model has been refreshed!'")
        except:
            os.system("ERROR!")
                

        
        
        
        
    
    
    
    

            
            
    
    
        








# urdf = ""    
        
# def geturdf():
#     urdf = rospy.get_param("/robot_description")
#     try:
#         f = open("temp.urdf", "w")
#         f.write(urdf)
#         print("[LOG] Temporary urdf file created!")
#     except Exception as e:
#         print(e)


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
        
