import sys
import os
import vtk
from PyQt5 import QtWidgets
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

class MinimalSTLViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Minimal STL Viewer")
        self.setGeometry(100, 100, 1000, 700)
        
        # Create central widget
        central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(central_widget)
        
        # Create layout
        layout = QtWidgets.QVBoxLayout(central_widget)
        
        # Create a frame to hold the VTK widget
        vtk_frame = QtWidgets.QFrame()
        layout.addWidget(vtk_frame)
        
        # Create VTK widget
        vtk_layout = QtWidgets.QVBoxLayout(vtk_frame)
        self.vtk_widget = QVTKRenderWindowInteractor(vtk_frame)
        vtk_layout.addWidget(self.vtk_widget)
        
        # Create renderer
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.1, 0.1, 0.1)  # Dark background
        
        # Add renderer to the window
        render_window = self.vtk_widget.GetRenderWindow()
        render_window.AddRenderer(self.renderer)
        
        # Create interactor
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        
        # Create button frame
        button_frame = QtWidgets.QFrame()
        layout.addWidget(button_frame)
        button_layout = QtWidgets.QHBoxLayout(button_frame)
        
        # Load button
        self.load_button = QtWidgets.QPushButton("Load STL File")
        self.load_button.clicked.connect(self.load_stl)
        button_layout.addWidget(self.load_button)
        
        # Reset view button
        self.reset_button = QtWidgets.QPushButton("Reset View")
        self.reset_button.clicked.connect(self.reset_view)
        button_layout.addWidget(self.reset_button)
        
        # Status label
        self.status_label = QtWidgets.QLabel("Ready. Click 'Load STL File' to begin.")
        layout.addWidget(self.status_label)
        
        # Initialize interactor
        self.interactor.Initialize()
        self.interactor.Start()
        
        # Add initial text to renderer
        self.text_actor = vtk.vtkTextActor()
        self.text_actor.SetInput("Please load an STL file")
        self.text_actor.SetPosition(10, 10)
        self.text_actor.GetTextProperty().SetFontSize(14)
        self.text_actor.GetTextProperty().SetColor(1.0, 1.0, 1.0)
        self.renderer.AddActor(self.text_actor)
        
        # Set up basic lighting
        light = vtk.vtkLight()
        light.SetPosition(0, 10, 10)
        light.SetColor(1, 1, 1)
        light.SetIntensity(1.0)
        self.renderer.AddLight(light)
        
        # Add a second light from the other side
        light2 = vtk.vtkLight()
        light2.SetPosition(0, -10, -10)
        light2.SetColor(0.6, 0.6, 0.6)
        light2.SetIntensity(0.8)
        self.renderer.AddLight(light2)
        
    def load_stl(self):
        """Load an STL file and display it"""
        try:
            # Open file dialog
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Open STL File", "", "STL Files (*.stl)"
            )
            
            if not filename:
                return
                
            self.status_label.setText(f"Loading: {os.path.basename(filename)}")
            
            # Read the STL file
            reader = vtk.vtkSTLReader()
            reader.SetFileName(filename)
            reader.Update()
            
            # Print some info about the model
            data = reader.GetOutput()
            num_cells = data.GetNumberOfCells()
            num_points = data.GetNumberOfPoints()
            bounds = data.GetBounds()
            
            print(f"Model loaded: {os.path.basename(filename)}")
            print(f"  Points: {num_points}")
            print(f"  Cells: {num_cells}")
            print(f"  Bounds: {bounds}")
            
            # Create mapper
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(reader.GetOutputPort())
            
            # Create actor
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.9, 0.9, 0.9)  # Light gray
            
            # Remove any existing actors
            self.renderer.RemoveAllViewProps()
            
            # Add the actor to the scene
            self.renderer.AddActor(actor)
            
            # Reset camera
            self.reset_view()
            
            self.status_label.setText(f"Loaded: {os.path.basename(filename)} - {num_points} points, {num_cells} cells")
            
        except Exception as e:
            import traceback
            error_details = traceback.format_exc()
            print(f"Error: {str(e)}")
            print(f"Details: {error_details}")
            self.status_label.setText(f"Error: {str(e)}")
            
    def reset_view(self):
        """Reset the camera view"""
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MinimalSTLViewer()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()