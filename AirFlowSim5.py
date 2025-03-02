import sys
import os
import vtk
import numpy as np
from PyQt5 import QtWidgets, QtCore
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

class AeroStreamlinesViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Aerodynamic Streamlines Viewer")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize variables
        self.model = None
        self.model_actor = None
        self.axes_actor = None
        self.streamline_actors = []
        self.animation_active = False
        self.airspeed = 50  # Default 50%
        self.text_actor = None
        self.vector_field = None
        self.locator = None
        
        # Create main layout
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QHBoxLayout(main_widget)
        
        # VTK view area (left side)
        vtk_frame = QtWidgets.QFrame()
        vtk_layout = QtWidgets.QVBoxLayout(vtk_frame)
        self.vtk_widget = QVTKRenderWindowInteractor(vtk_frame)
        vtk_layout.addWidget(self.vtk_widget)
        main_layout.addWidget(vtk_frame, 3)  # Takes 75% of width
        
        # Control panel (right side)
        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control_panel)
        main_layout.addWidget(control_panel, 1)  # Takes 25% of width
        
        # Model controls group
        model_group = QtWidgets.QGroupBox("Model")
        model_layout = QtWidgets.QVBoxLayout(model_group)
        
        # Load button
        load_button = QtWidgets.QPushButton("Load STL Model")
        load_button.clicked.connect(self.load_stl)
        model_layout.addWidget(load_button)
        
        # Model info display
        self.model_info = QtWidgets.QLabel("No model loaded")
        model_layout.addWidget(self.model_info)
        
        control_layout.addWidget(model_group)
        
        # Airflow controls group
        airflow_group = QtWidgets.QGroupBox("Airflow Settings")
        airflow_layout = QtWidgets.QVBoxLayout(airflow_group)
        
        # Airspeed slider
        airspeed_layout = QtWidgets.QHBoxLayout()
        airspeed_layout.addWidget(QtWidgets.QLabel("Airspeed:"))
        self.airspeed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.airspeed_slider.setRange(10, 100)
        self.airspeed_slider.setValue(self.airspeed)
        self.airspeed_slider.valueChanged.connect(self.update_airspeed)
        airspeed_layout.addWidget(self.airspeed_slider)
        self.airspeed_value = QtWidgets.QLabel(f"{self.airspeed}%")
        airspeed_layout.addWidget(self.airspeed_value)
        airflow_layout.addLayout(airspeed_layout)
        
        # Visualization type
        viz_layout = QtWidgets.QHBoxLayout()
        viz_layout.addWidget(QtWidgets.QLabel("Visualization:"))
        self.viz_type = QtWidgets.QComboBox()
        self.viz_type.addItems(["Streamlines", "Particles"])
        viz_layout.addWidget(self.viz_type)
        airflow_layout.addLayout(viz_layout)
        
        # Streamline density
        density_layout = QtWidgets.QHBoxLayout()
        density_layout.addWidget(QtWidgets.QLabel("Density:"))
        self.density_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.density_slider.setRange(10, 100)
        self.density_slider.setValue(50)
        density_layout.addWidget(self.density_slider)
        airflow_layout.addLayout(density_layout)
        
        # Animation control
        self.animation_button = QtWidgets.QPushButton("Start Animation")
        self.animation_button.clicked.connect(self.toggle_animation)
        self.animation_button.setEnabled(False)  # Disabled until model loaded
        airflow_layout.addWidget(self.animation_button)
        
        control_layout.addWidget(airflow_group)
        
        # View controls group
        view_group = QtWidgets.QGroupBox("View Controls")
        view_layout = QtWidgets.QVBoxLayout(view_group)
        
        # Camera controls
        zoom_layout = QtWidgets.QHBoxLayout()
        zoom_in_button = QtWidgets.QPushButton("Zoom In")
        zoom_in_button.clicked.connect(self.zoom_in)
        zoom_layout.addWidget(zoom_in_button)
        
        zoom_out_button = QtWidgets.QPushButton("Zoom Out")
        zoom_out_button.clicked.connect(self.zoom_out)
        zoom_layout.addWidget(zoom_out_button)
        view_layout.addLayout(zoom_layout)
        
        fit_button = QtWidgets.QPushButton("Zoom to Fit")
        fit_button.clicked.connect(self.zoom_to_fit)
        view_layout.addWidget(fit_button)
        
        # Front/side view buttons
        view_buttons_layout = QtWidgets.QHBoxLayout()
        front_button = QtWidgets.QPushButton("Front View")
        front_button.clicked.connect(lambda: self.set_view("front"))
        view_buttons_layout.addWidget(front_button)
        
        side_button = QtWidgets.QPushButton("Side View")
        side_button.clicked.connect(lambda: self.set_view("side"))
        view_buttons_layout.addWidget(side_button)
        view_layout.addLayout(view_buttons_layout)
        
        control_layout.addWidget(view_group)
        
        # Status bar
        self.statusBar().showMessage("Ready")
        
        # Add stretcher to push everything up
        control_layout.addStretch()
        
        # Set up renderer
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.1, 0.1, 0.2)  # Dark blue background
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        
        # Set up interactor
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        self.interactor.Initialize()
        
        # Setup initial scene
        self.setup_lighting()
        self.add_axes()
        
        # Add initial text
        self.add_text("Load an STL model to begin")
        
        # Setup animation timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_animation)
        
        # For handling periodic updates
        self.last_streamline_creation = 0
        
    def add_text(self, text):
        """Add text overlay to the scene"""
        if self.text_actor:
            self.renderer.RemoveActor(self.text_actor)
            
        text_actor = vtk.vtkTextActor()
        text_actor.SetInput(text)
        text_actor.GetTextProperty().SetFontSize(16)
        text_actor.GetTextProperty().SetColor(1.0, 1.0, 1.0)
        text_actor.SetPosition(10, 10)
        self.renderer.AddActor(text_actor)
        self.text_actor = text_actor
    
    def setup_lighting(self):
        """Set up bright lighting from multiple angles"""
        # Remove existing lights
        self.renderer.RemoveAllLights()
        
        # Add front light
        front_light = vtk.vtkLight()
        front_light.SetPosition(1000, 0, 0)
        front_light.SetFocalPoint(0, 0, 0)
        front_light.SetIntensity(0.8)
        front_light.SetColor(1.0, 1.0, 1.0)
        self.renderer.AddLight(front_light)
        
        # Add top light
        top_light = vtk.vtkLight()
        top_light.SetPosition(0, 0, 1000)
        top_light.SetFocalPoint(0, 0, 0)
        top_light.SetIntensity(0.6)
        top_light.SetColor(1.0, 1.0, 1.0)
        self.renderer.AddLight(top_light)
        
        # Add side light
        side_light = vtk.vtkLight()
        side_light.SetPosition(0, 1000, 0)
        side_light.SetFocalPoint(0, 0, 0)
        side_light.SetIntensity(0.6)
        side_light.SetColor(1.0, 1.0, 1.0)
        self.renderer.AddLight(side_light)
    
    def add_axes(self):
        """Add coordinate axes to the scene"""
        if self.axes_actor:
            self.renderer.RemoveActor(self.axes_actor)
        
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(100, 100, 100)
        axes.SetXAxisLabelText("X")
        axes.SetYAxisLabelText("Y")
        axes.SetZAxisLabelText("Z")
        
        # Make labels larger
        axes.GetXAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone()
        axes.GetXAxisCaptionActor2D().GetTextActor().GetTextProperty().SetFontSize(16)
        axes.GetYAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone()
        axes.GetYAxisCaptionActor2D().GetTextActor().GetTextProperty().SetFontSize(16)
        axes.GetZAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone()
        axes.GetZAxisCaptionActor2D().GetTextActor().GetTextProperty().SetFontSize(16)
        
        # Add to renderer
        self.renderer.AddActor(axes)
        self.axes_actor = axes
    
    def load_stl(self):
        """Load an STL file with progress dialog"""
        try:
            # Open file dialog
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Open STL File", "", "STL Files (*.stl)"
            )
            
            if not filename:
                return
            
            # Create progress dialog
            progress = QtWidgets.QProgressDialog("Loading model...", "Cancel", 0, 100, self)
            progress.setWindowTitle("Loading STL Model")
            progress.setWindowModality(QtCore.Qt.WindowModal)
            progress.setValue(0)
            progress.show()
            QtWidgets.QApplication.processEvents()
            
            self.statusBar().showMessage(f"Loading {os.path.basename(filename)}...")
            
            # Read the STL file
            reader = vtk.vtkSTLReader()
            reader.SetFileName(filename)
            progress.setValue(10)
            QtWidgets.QApplication.processEvents()
            reader.Update()
            progress.setValue(20)
            QtWidgets.QApplication.processEvents()
            
            # Get model data
            stl_polydata = reader.GetOutput()
            
            if stl_polydata.GetNumberOfPoints() == 0:
                progress.close()
                self.statusBar().showMessage("Error: STL file contains no points")
                return
            
            # Get model info
            num_points = stl_polydata.GetNumberOfPoints()
            num_cells = stl_polydata.GetNumberOfCells()
            original_bounds = stl_polydata.GetBounds()
            
            print(f"Model statistics:")
            print(f"  Points: {num_points}")
            print(f"  Cells: {num_cells}")
            print(f"  Bounds: {original_bounds}")
            
            progress.setValue(30)
            progress.setLabelText("Generating surface normals...")
            QtWidgets.QApplication.processEvents()
            
            # Generate normals if they don't exist
            if not stl_polydata.GetPointData().GetNormals():
                print("Generating surface normals...")
                normals = vtk.vtkPolyDataNormals()
                normals.SetInputData(stl_polydata)
                normals.ComputePointNormalsOn()
                normals.ComputeCellNormalsOn()
                normals.SplittingOff()
                normals.ConsistencyOn()
                normals.Update()
                stl_polydata = normals.GetOutput()
            
            progress.setValue(40)
            progress.setLabelText("Processing model...")
            QtWidgets.QApplication.processEvents()
            
            # Process the model (center and scale)
            self.model = self.process_model(stl_polydata)
            
            progress.setValue(50)
            progress.setLabelText("Setting up visualization...")
            QtWidgets.QApplication.processEvents()
            
            # Create lookup table for model pressure coloring
            lut = vtk.vtkLookupTable()
            lut.SetNumberOfTableValues(256)
            lut.SetHueRange(0.667, 0.0)  # Blue to red
            lut.SetSaturationRange(1.0, 1.0)
            lut.SetValueRange(1.0, 1.0)
            lut.SetAlphaRange(1.0, 1.0)
            lut.SetTableRange(0.5, 1.5)  # Pressure range
            lut.Build()
            
            progress.setValue(60)
            QtWidgets.QApplication.processEvents()
            
            # Create mapper
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(self.model)
            mapper.SetScalarModeToUsePointData()
            mapper.SetScalarRange(0.5, 1.5)
            mapper.SetLookupTable(lut)
            
            # Create actor
            if self.model_actor:
                self.renderer.RemoveActor(self.model_actor)
            
            self.model_actor = vtk.vtkActor()
            self.model_actor.SetMapper(mapper)
            self.model_actor.GetProperty().SetSpecular(0.3)
            self.model_actor.GetProperty().SetSpecularPower(20)
            
            # Clear scene and add model
            self.renderer.RemoveAllViewProps()
            self.renderer.AddActor(self.model_actor)
            self.add_axes()
            
            progress.setValue(70)
            progress.setLabelText("Building spatial lookup structures...")
            QtWidgets.QApplication.processEvents()
            
            # Create cell locator for the model (do this once)
            if self.locator:
                self.locator = None
            self.locator = vtk.vtkCellLocator()
            self.locator.SetDataSet(self.model)
            self.locator.BuildLocator()
            
            # Create implicit function for inside/outside testing
            self.model_implicit = vtk.vtkImplicitPolyDataDistance()
            self.model_implicit.SetInput(self.model)
            
            progress.setValue(80)
            progress.setLabelText("Setting up flow field...")
            QtWidgets.QApplication.processEvents()
            
            # Create the velocity field
            self.setup_velocity_field()
            
            progress.setValue(90)
            QtWidgets.QApplication.processEvents()
            
            # Enable animation button
            self.animation_button.setEnabled(True)
            
            # Update UI
            self.model_info.setText(f"Model: {os.path.basename(filename)}\nPoints: {num_points}\nCells: {num_cells}")
            
            # Set view to show model
            self.zoom_to_fit()
            self.set_view("side")  # Start with side view for aerodynamics
            
            progress.setValue(100)
            progress.close()
            
            self.statusBar().showMessage(f"Loaded: {os.path.basename(filename)}")
            
        except Exception as e:
            import traceback
            print(f"Error loading STL: {str(e)}")
            print(traceback.format_exc())
            if 'progress' in locals():
                progress.close()
            self.statusBar().showMessage(f"Error: {str(e)}")
    
    def process_model(self, polydata):
        """Center and scale the model to fit well in view, and add surface coloring"""
        # Get original bounds
        bounds = polydata.GetBounds()
        
        # Calculate center
        center = [
            (bounds[0] + bounds[1]) / 2,
            (bounds[2] + bounds[3]) / 2,
            (bounds[4] + bounds[5]) / 2
        ]
        
        # Calculate dimensions
        dimensions = [
            bounds[1] - bounds[0],
            bounds[3] - bounds[2],
            bounds[5] - bounds[4]
        ]
        max_dim = max(dimensions)
        
        # Create transform to center and scale
        transform = vtk.vtkTransform()
        transform.Translate(-center[0], -center[1], -center[2])  # Center at origin
        
        # Scale to standard size
        scale_factor = 200.0 / max_dim if max_dim > 0 else 1.0
        transform.Scale(scale_factor, scale_factor, scale_factor)
        
        # Apply transform
        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetInputData(polydata)
        transform_filter.SetTransform(transform)
        transform_filter.Update()
        
        # Get transformed model
        model = transform_filter.GetOutput()
        
        # Add pressure field to model surface
        self.add_surface_pressure(model)
        
        return model
        
    def add_surface_pressure(self, model):
        """Add pressure field to model surface for visualization"""
        if not model:
            return
            
        # Add a scalar field for pressure visualization
        num_points = model.GetNumberOfPoints()
        pressure = vtk.vtkFloatArray()
        pressure.SetName("surface_pressure")
        pressure.SetNumberOfComponents(1)
        pressure.SetNumberOfTuples(num_points)
        
        # Get model bounds and center
        bounds = model.GetBounds()
        center = [
            (bounds[0] + bounds[1]) / 2,
            (bounds[2] + bounds[3]) / 2,
            (bounds[4] + bounds[5]) / 2
        ]
        
        # Calculate approximate pressure for each point based on its position
        for i in range(num_points):
            point = model.GetPoint(i)
            
            # Calculate position relative to center
            rel_pos = [
                point[0] - center[0],
                point[1] - center[1], 
                point[2] - center[2]
            ]
            
            # Calculate surface normal (approximate)
            point_id = model.FindPoint(point)
            normals = model.GetPointData().GetNormals()
            normal = normals.GetTuple3(point_id) if normals else [1, 0, 0]
            
            # Normalize normal
            norm_mag = np.sqrt(sum(n*n for n in normal))
            if norm_mag > 0.0001:
                normal = [n/norm_mag for n in normal]
            
            # Front-facing surfaces (high pressure)
            # Dot product of normal with x-axis (flow direction)
            front_facing = -normal[0]  # Negative because normals point outward
            
            # Calculate pressure:
            # 1. High pressure at front stagnation points (red)
            # 2. Low pressure on sides/top where flow accelerates (blue)
            # 3. Medium pressure at rear
            
            # Base pressure
            pressure_val = 1.0  # Neutral
            
            if front_facing > 0.7:
                # Front-facing surfaces (high pressure)
                pressure_val = 1.0 + front_facing * 0.5
            elif abs(normal[1]) > 0.7 or abs(normal[2]) > 0.7:
                # Side/top/bottom surfaces (low pressure)
                # Lower pressure where flow accelerates
                side_factor = max(abs(normal[1]), abs(normal[2]))
                pressure_val = 1.0 - side_factor * 0.4
            elif normal[0] > 0.7:
                # Rear-facing surface (medium-low pressure)
                pressure_val = 0.7
            
            # Add some variation based on position
            # Front should be higher pressure, sides/top lower pressure
            x_rel = (point[0] - bounds[0]) / (bounds[1] - bounds[0])
            
            # Higher pressure at front, lower at sides
            if x_rel < 0.3:  # Front section
                pressure_val += 0.1 * (1.0 - x_rel/0.3)
            
            # Set pressure value
            pressure.SetTuple1(i, pressure_val)
        
        # Add pressure field to model
        model.GetPointData().SetScalars(pressure)
    
    def setup_velocity_field(self):
        """Create the velocity field once and store it"""
        if not self.model:
            return
            
        # Get model bounds
        bounds = self.model.GetBounds()
        
        # Calculate domain with margin - expanded for better coverage
        x_min = bounds[0] - 350  # Upstream distance
        x_max = bounds[1] + 450  # Extended downstream for wake
        y_min = bounds[2] - 250
        y_max = bounds[3] + 250
        z_min = bounds[4] - 250
        z_max = bounds[5] + 250
        
        bounds_with_margin = [x_min-100, x_max+100, y_min-100, y_max+100, z_min-100, z_max+100]
        
        # Create uniform velocity vector field with higher resolution
        self.vector_field = vtk.vtkImageData()
        self.vector_field.SetDimensions(40, 40, 40)  # Higher resolution for more detail
        self.vector_field.SetExtent(0, 39, 0, 39, 0, 39)
        self.vector_field.SetOrigin(bounds_with_margin[0], bounds_with_margin[2], bounds_with_margin[4])
        self.vector_field.SetSpacing(
            (bounds_with_margin[1] - bounds_with_margin[0]) / 39,
            (bounds_with_margin[3] - bounds_with_margin[2]) / 39,
            (bounds_with_margin[5] - bounds_with_margin[4]) / 39
        )
        
        # Create velocity vectors (u, v, w) components
        self.update_vector_field()
    
    def update_vector_field(self):
        """Update the vector field based on current airspeed and calculate pressure"""
        if not self.vector_field or not self.model:
            return
            
        # Scale speed based on airspeed setting
        speed_factor = self.airspeed / 50.0  # 1.0 at 50% airspeed
        
        # Create velocity vectors and pressure field
        vectors = vtk.vtkFloatArray()
        vectors.SetNumberOfComponents(3)  # 3D vector
        vectors.SetName("velocity")
        
        # Add pressure scalar field
        pressure = vtk.vtkFloatArray()
        pressure.SetNumberOfComponents(1)
        pressure.SetName("pressure")
        
        # Calculate number of points in the vector field
        dim = self.vector_field.GetDimensions()
        num_points = dim[0] * dim[1] * dim[2]
        
        # Pre-allocate the arrays
        vectors.SetNumberOfTuples(num_points)
        pressure.SetNumberOfTuples(num_points)
        
        # For efficiency, get the bounds once
        bounds = self.model.GetBounds()
        model_center = [
            (bounds[0] + bounds[1]) / 2,
            (bounds[2] + bounds[3]) / 2,
            (bounds[4] + bounds[5]) / 2
        ]
        
        # Reference pressure and density (arbitrary units for visualization)
        p_ref = 1.0
        density = 1.0
        
        # Base velocity magnitude (free stream)
        v_base = 1.0 * speed_factor
        
        # Create a more realistic flow field with better surface interaction
        for i in range(num_points):
            # Default values - base flow in x direction
            u = v_base
            v = 0.0
            w = 0.0
            
            # Get point coordinates
            point_id = i
            point = self.vector_field.GetPoint(point_id)
            
            # Test if point is inside the model - set zero velocity if inside
            if self.model_implicit:
                # Get signed distance from model surface (negative inside, positive outside)
                dist = self.model_implicit.EvaluateFunction(point)
                
                # If point is inside or very close to model, zero velocity
                if dist <= 0.1:  # Inside or very close to surface
                    vectors.SetTuple3(i, 0.0, 0.0, 0.0)
                    pressure.SetTuple1(i, p_ref)
                    continue
            
            # Get closest point on model and distance
            closest_point = [0, 0, 0]
            cell_id = vtk.mutable(0)
            sub_id = vtk.mutable(0)
            dist2 = vtk.mutable(0.0)
            
            # Use the pre-built locator
            pressure_value = p_ref  # Default reference pressure
            
            if self.locator:
                self.locator.FindClosestPoint(point, closest_point, cell_id, sub_id, dist2)
                distance = np.sqrt(dist2.get())
                
                # Enhanced flow behavior around the model with realistic pressure zones
                if distance < 150:
                    # Calculate direction from point to closest point on model
                    direction = [
                        closest_point[0] - point[0],
                        closest_point[1] - point[1], 
                        closest_point[2] - point[2]
                    ]
                    
                    # Normalize direction (safely)
                    mag = np.sqrt(sum(d*d for d in direction))
                    if mag > 0.0001:
                        direction = [d/mag for d in direction]
                        
                        # Front of the model (stagnation region) - high pressure
                        if point[0] < model_center[0] and distance < 50:
                            # Check if this is the front-facing area
                            if direction[0] > 0.7:  # Mainly facing the flow
                                # Stagnation point/region - highest pressure
                                stagnation_factor = (1.0 - distance/50.0) * (direction[0])
                                pressure_value = p_ref + 0.5 * density * v_base**2 * stagnation_factor
                                
                                # Flow slows down significantly near stagnation point
                                u *= max(0.1, 1.0 - stagnation_factor)
                                v += direction[1] * stagnation_factor * 0.2
                                w += direction[2] * stagnation_factor * 0.2
                        
                        # Very close to surface - boundary layer effect
                        if distance < 15:
                            # Slow down significantly near surface (boundary layer)
                            boundary_factor = min(1.0, distance / 15.0)
                            u *= boundary_factor**2  # Quadratic slowdown
                            
                            # Side/top/bottom of model - low pressure areas
                            # Check if this is along the sides/top/bottom of the model
                            if abs(direction[0]) < 0.5 and abs(direction[1]) > 0.5 or abs(direction[2]) > 0.5:
                                # Acceleration zone (sides/top/bottom) - lower pressure
                                accel_factor = (1.0 - distance/15.0) * (1.0 - abs(direction[0]))
                                
                                # Calculate surface-following flow - higher speed at sides/top
                                speed_increase = 1.3 + accel_factor
                                
                                # Flow along the body - ensure it follows the surface and doesn't penetrate
                                # Project velocity onto surface by removing normal component
                                normal_component = direction[0] * u + direction[1] * v + direction[2] * w
                                
                                # Subtract normal component and add tangential component
                                u = (u - normal_component * direction[0]) * speed_increase
                                v = (v - normal_component * direction[1]) * speed_increase
                                w = (w - normal_component * direction[2]) * speed_increase
                                
                                # Ensure flow doesn't point into the model
                                dot_product = u * direction[0] + v * direction[1] + w * direction[2]
                                if dot_product > 0:  # If flow points toward model
                                    # Reverse the component toward the model
                                    u -= 2 * dot_product * direction[0]
                                    v -= 2 * dot_product * direction[1]
                                    w -= 2 * dot_product * direction[2]
                                
                                # Pressure drops due to higher velocity (Bernoulli's principle)
                                pressure_value = p_ref - 0.5 * density * (speed_increase - 1.0) * v_base**2
                        
                        # Medium distance - transition zone
                        elif distance < 50:
                            # Smoother transition of flow around body
                            trans_factor = (50.0 - distance) / 35.0
                            
                            # Flow deflection based on position relative to model
                            side_deflection = 1.0 - abs(direction[0])
                            if side_deflection > 0.3:  # Not directly in front or behind
                                # Increase speed around sides/top/bottom
                                u *= 1.0 + side_deflection * 0.3
                                
                                # Deflect flow around model - ensure it doesn't point into the model
                                # Calculate deflection components
                                deflect_v = -direction[1] * trans_factor * 0.4
                                deflect_w = -direction[2] * trans_factor * 0.4
                                
                                # Add deflection components
                                v += deflect_v
                                w += deflect_w
                                
                                # Check if resulting flow points toward model and correct if needed
                                dot_product = u * direction[0] + v * direction[1] + w * direction[2]
                                if dot_product > 0:  # Flow points toward model
                                    # Add stronger deflection to prevent penetration
                                    v -= dot_product * direction[1] * 1.2
                                    w -= dot_product * direction[2] * 1.2
                                
                                # Lower pressure in acceleration zones
                                pressure_value = p_ref - 0.2 * side_deflection * v_base**2
                
                # Wake effect behind the model
                if point[0] > model_center[0]:
                    # Check if point is in the potential wake zone
                    dx = point[0] - model_center[0]
                    
                    # Wake width starts narrow and expands with distance
                    wake_width = 50.0 + dx * 0.3  # Wake expands gradually
                    
                    # Calculate distance from wake centerline
                    dy = abs(point[1] - model_center[1])
                    dz = abs(point[2] - model_center[2])
                    
                    # Define wake boundary
                    if dy < wake_width and dz < wake_width and dx < 400:
                        # Calculate wake intensity (strongest near the model and centerline)
                        centerline_dist = np.sqrt(dy**2 + dz**2) / wake_width
                        wake_intensity = (1.0 - min(1.0, dx/400.0)) * (1.0 - centerline_dist)
                        
                        # Reduce velocity in wake
                        u *= max(0.3, 1.0 - wake_intensity * 0.7)
                        
                        # Low pressure wake region
                        pressure_value = p_ref - 0.3 * wake_intensity * v_base**2
                        
                        # Add some turbulence/vorticity in the wake
                        if centerline_dist < 0.7:  # More turbulence near wake centerline
                            turbulence = wake_intensity * 0.25
                            
                            # Create counter-rotating vortices
                            vortex_factor = 0.2 * wake_intensity
                            v_sign = 1 if point[1] > model_center[1] else -1
                            w_sign = 1 if point[2] > model_center[2] else -1
                            
                            # Add vortex rotation effects
                            v += w_sign * vortex_factor * (point[2] - model_center[2]) / wake_width
                            w -= v_sign * vortex_factor * (point[1] - model_center[1]) / wake_width
                            
                            # Add small-scale turbulence
                            phase1 = point[0]/20.0 + point[1]/15.0 + point[2]/30.0
                            phase2 = point[0]/15.0 - point[1]/25.0 + point[2]/20.0
                            v += np.sin(phase1) * turbulence
                            w += np.sin(phase2) * turbulence
            
            # Calculate velocity magnitude for pressure calculation
            vel_mag = np.sqrt(u**2 + v**2 + w**2)
            
            # Store vectors and pressure
            vectors.SetTuple3(i, u, v, w)
            pressure.SetTuple1(i, pressure_value)
        
        # Add vectors and pressure to field
        self.vector_field.GetPointData().SetVectors(vectors)
        self.vector_field.GetPointData().SetScalars(pressure)
    
    def update_airspeed(self, value):
        """Update airspeed value and refresh visualization"""
        self.airspeed = value
        self.airspeed_value.setText(f"{value}%")
        
        # Update vector field
        if self.vector_field:
            self.update_vector_field()
        
        # If animation is active, no need to explicitly update streamlines
        # as that will happen in the next animation frame
    
    def clear_streamlines(self):
        """Remove all streamline actors"""
        for actor in self.streamline_actors:
            self.renderer.RemoveActor(actor)
        self.streamline_actors = []
    
    def create_streamlines(self):
        """Create streamlines based on current settings"""
        if not self.model or not self.vector_field:
            return
        
        # Apply density settings
        density_factor = self.density_slider.value() / 50.0  # 1.0 at 50% density
        
        # Get model bounds
        bounds = self.model.GetBounds()
        
        # Calculate domain
        x_min = bounds[0] - 300  # Upstream distance
        x_max = bounds[1] + 200  # Downstream distance
        y_min = bounds[2] - 200
        y_max = bounds[3] + 200
        z_min = bounds[4] - 250
        z_max = bounds[5] + 250
        
        # Decide on number of seed points - increased for better coverage
        base_points = 20  # Much higher density for realistic CFD visualization
        num_y_points = max(10, int(base_points * density_factor))
        num_z_points = max(10, int(base_points * density_factor))
        
        # Create points in a more distributed pattern around the model
        seed_points = vtk.vtkPoints()
        
        # Create seed points only outside the model
        # Front plane (upstream - this is a safe area for seeds, always outside model)
        for i in range(num_y_points):
            y = y_min + (i / max(1, num_y_points-1)) * (y_max - y_min)
            for j in range(num_z_points):
                z = z_min + (j / max(1, num_z_points-1)) * (z_max - z_min)
                seed_points.InsertNextPoint(x_min, y, z)
        
        # Top plane - with clearance from model
        top_z = bounds[5] + 20  # Ensure some clearance
        for i in range(int(num_y_points/2)):
            y = y_min + (i / max(1, int(num_y_points/2)-1)) * (y_max - y_min)
            for j in range(int(num_z_points/2)):
                x = x_min + (j / max(1, int(num_z_points/2)-1)) * (x_max - x_min)
                seed_points.InsertNextPoint(x, y, top_z)
        
        # Bottom plane - with clearance
        bottom_z = bounds[4] - 20
        for i in range(int(num_y_points/2)):
            y = y_min + (i / max(1, int(num_y_points/2)-1)) * (y_max - y_min)
            for j in range(int(num_z_points/2)):
                x = x_min + (j / max(1, int(num_z_points/2)-1)) * (x_max - x_min)
                seed_points.InsertNextPoint(x, y, bottom_z)
        
        # Left side plane - with clearance
        left_y = bounds[2] - 20
        for i in range(int(num_y_points/2)):
            x = x_min + (i / max(1, int(num_y_points/2)-1)) * (x_max - x_min)
            for j in range(int(num_z_points/2)):
                z = z_min + (j / max(1, int(num_z_points/2)-1)) * (z_max - z_min)
                seed_points.InsertNextPoint(x, left_y, z)
        
        # Right side plane - with clearance
        right_y = bounds[3] + 20
        for i in range(int(num_y_points/2)):
            x = x_min + (i / max(1, int(num_y_points/2)-1)) * (x_max - x_min)
            for j in range(int(num_z_points/2)):
                z = z_min + (j / max(1, int(num_z_points/2)-1)) * (z_max - z_min)
                seed_points.InsertNextPoint(x, right_y, z)
        
        # Create polydata for seeds
        seed_polydata = vtk.vtkPolyData()
        seed_polydata.SetPoints(seed_points)
        
        # Create streamline source
        streamline = vtk.vtkStreamTracer()
        streamline.SetInputData(self.vector_field)
        streamline.SetSourceData(seed_polydata)
        streamline.SetMaximumPropagation(1000)
        streamline.SetIntegrationDirectionToForward()
        streamline.SetIntegratorTypeToRungeKutta45()
        streamline.SetIntegrationStepUnit(vtk.vtkStreamTracer.CELL_LENGTH_UNIT)
        streamline.SetInitialIntegrationStep(0.5)
        streamline.SetMinimumIntegrationStep(0.1)
        streamline.SetMaximumIntegrationStep(1.0)
        
        # DO NOT set surface streamlines (causes errors with 3D cells)
        # streamline.SetSurfaceStreamlines(True) - this line is removed
        streamline.SetComputeVorticity(True)
        
        streamline.Update()
        
        # Create a lookup table for pressure-based coloring
        lut = vtk.vtkLookupTable()
        lut.SetNumberOfTableValues(256)
        lut.SetHueRange(0.667, 0.0)  # Blue to red
        lut.SetSaturationRange(1.0, 1.0)
        lut.SetValueRange(1.0, 1.0)
        lut.SetAlphaRange(1.0, 1.0)
        lut.SetTableRange(0.5, 1.5)  # Pressure range
        lut.Build()
        
        # Choose visualization based on selected type
        if self.viz_type.currentText() == "Streamlines":
            # Create mapper with streamlines
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(streamline.GetOutputPort())
            mapper.SetScalarModeToUsePointFieldData()
            mapper.SelectColorArray("pressure")
            mapper.SetLookupTable(lut)
            mapper.UseLookupTableScalarRangeOn()
            
            # Create actor for streamlines
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            
            # Set line properties - thin lines for CFD style
            actor.GetProperty().SetLineWidth(1.0)
            actor.GetProperty().SetRepresentationToWireframe()
            
            # Add to renderer
            self.renderer.AddActor(actor)
            self.streamline_actors.append(actor)
            
        else:  # Particles
            # Use points for particles instead of arrows
            # We'll use a glyph but with small spheres instead of arrows
            sphere = vtk.vtkSphereSource()
            sphere.SetRadius(0.8)  # Small dots
            sphere.SetPhiResolution(8)  # Lower resolution for performance
            sphere.SetThetaResolution(8)
            
            glyph = vtk.vtkGlyph3D()
            glyph.SetInputConnection(streamline.GetOutputPort())
            glyph.SetSourceConnection(sphere.GetOutputPort())
            glyph.SetScaleModeToDataScalingOff()
            glyph.SetColorModeToColorByScalar()
            glyph.SetScaleFactor(1.0)
            glyph.Update()
            
            # Create mapper
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(glyph.GetOutputPort())
            mapper.SetScalarModeToUsePointFieldData()
            mapper.SelectColorArray("pressure")
            mapper.SetLookupTable(lut)
            mapper.UseLookupTableScalarRangeOn()
            
            # Create actor
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            
            # Add to renderer
            self.renderer.AddActor(actor)
            self.streamline_actors.append(actor)
        
        # Render the scene
        QtWidgets.QApplication.processEvents()
        self.vtk_widget.GetRenderWindow().Render()
        
        # Create streamline source
        streamline = vtk.vtkStreamTracer()
        streamline.SetInputData(self.vector_field)
        streamline.SetSourceData(seed_polydata)
        streamline.SetMaximumPropagation(1000)  # Long streamlines
        streamline.SetIntegrationDirectionToForward()
        streamline.SetIntegratorTypeToRungeKutta45()
        streamline.SetIntegrationStepUnit(vtk.vtkStreamTracer.CELL_LENGTH_UNIT)
        streamline.SetInitialIntegrationStep(0.5)  # Larger steps for efficiency
        streamline.SetMinimumIntegrationStep(0.1)
        streamline.SetMaximumIntegrationStep(1.0)
        streamline.Update()
        
        # Choose visualization based on selected type
        if self.viz_type.currentText() == "Streamlines":
            # Create tube filter for streamlines
            tube_filter = vtk.vtkTubeFilter()
            tube_filter.SetInputConnection(streamline.GetOutputPort())
            tube_filter.SetRadius(1.5)  # Thicker tubes for visibility
            tube_filter.SetNumberOfSides(8)  # Reduced from 12 for efficiency
            tube_filter.SetVaryRadiusToVaryRadiusOff()
            tube_filter.Update()
            
            # Create mapper
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(tube_filter.GetOutputPort())
            
            # Create actor
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.1, 0.6, 1.0)  # Light blue
            
            # Add to renderer
            self.renderer.AddActor(actor)
            self.streamline_actors.append(actor)
            
        else:  # Particles
            # Use glyph for particles
            arrow = vtk.vtkArrowSource()
            arrow.SetTipResolution(8)  # Reduced from 16
            arrow.SetTipRadius(0.1)
            arrow.SetTipLength(0.35)
            arrow.SetShaftResolution(8)  # Reduced from 16
            arrow.SetShaftRadius(0.03)
            
            glyph = vtk.vtkGlyph3D()
            glyph.SetInputConnection(streamline.GetOutputPort())
            glyph.SetSourceConnection(arrow.GetOutputPort())
            glyph.SetVectorModeToUseVector()
            glyph.SetScaleModeToScaleByVector()
            glyph.SetScaleFactor(5.0)  # Scale for visibility
            glyph.OrientOn()
            glyph.Update()
            
            # Create mapper
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(glyph.GetOutputPort())
            
            # Create actor
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.1, 0.6, 1.0)  # Light blue
            
            # Add to renderer
            self.renderer.AddActor(actor)
            self.streamline_actors.append(actor)
        
        # Render the scene
        QtWidgets.QApplication.processEvents()  # Allow UI to update
        self.vtk_widget.GetRenderWindow().Render()
    
    def toggle_animation(self):
        """Start or stop the animation"""
        if not self.animation_active:
            # Start animation
            self.animation_active = True
            self.animation_button.setText("Stop Animation")
            
            # Clear any existing streamlines first
            self.clear_streamlines()
            
            # Create initial streamlines
            try:
                self.create_streamlines()
                
                # Start the timer
                self.timer.start(50)  # 20 FPS
                
                self.statusBar().showMessage("Animation started")
                
            except Exception as e:
                import traceback
                self.animation_active = False
                self.animation_button.setText("Start Animation")
                print(f"Error starting animation: {str(e)}")
                print(traceback.format_exc())
                self.statusBar().showMessage(f"Error: {str(e)}")
            
        else:
            # Stop animation
            self.animation_active = False
            self.animation_button.setText("Start Animation")
            self.timer.stop()
            
            # Clear streamlines
            self.clear_streamlines()
            
            self.statusBar().showMessage("Animation stopped")
    
    def update_animation(self):
        """Update animation frame"""
        if not self.animation_active or not self.model:
            return
        
        try:
            # Use a copy of streamline_actors to avoid modification issues during iteration
            current_actors = self.streamline_actors.copy()
            actors_to_remove = []
            
            # Animate existing streamlines
            for actor in current_actors:
                # Get current position
                position = actor.GetPosition()
                
                # Move in x direction based on airspeed
                speed = self.airspeed / 50.0 * 2.0  # Scale speed
                actor.SetPosition(position[0] + speed, position[1], position[2])
                
                # If moved past end of domain, mark for removal
                bounds = self.model.GetBounds()
                if position[0] > bounds[1] + 300:
                    actors_to_remove.append(actor)
            
            # Remove actors after iteration
            for actor in actors_to_remove:
                self.renderer.RemoveActor(actor)
                if actor in self.streamline_actors:
                    self.streamline_actors.remove(actor)
            
            # Periodically add new streamlines (not every frame)
            current_time = QtCore.QTime.currentTime().msecsSinceStartOfDay()
            if (current_time - self.last_streamline_creation > 250 and 
                len(self.streamline_actors) < 10):  # Fewer actors for better performance
                self.create_streamlines()
                self.last_streamline_creation = current_time
            
            # Process events to keep UI responsive
            QtWidgets.QApplication.processEvents()
            
            # Render the scene
            self.vtk_widget.GetRenderWindow().Render()
            
        except Exception as e:
            import traceback
            print(f"Error in animation update: {str(e)}")
            print(traceback.format_exc())
            self.timer.stop()
            self.animation_active = False
            self.animation_button.setText("Start Animation")
            self.statusBar().showMessage(f"Animation error: {str(e)}")
    
    def zoom_in(self):
        """Zoom camera in"""
        camera = self.renderer.GetActiveCamera()
        camera.Zoom(1.5)  # Zoom in by 50%
        self.vtk_widget.GetRenderWindow().Render()
    
    def zoom_out(self):
        """Zoom camera out"""
        camera = self.renderer.GetActiveCamera()
        camera.Zoom(0.67)  # Zoom out by 33%
        self.vtk_widget.GetRenderWindow().Render()
    
    def zoom_to_fit(self):
        """Zoom to fit model in view"""
        if not self.model:
            return
        
        # Get model bounds
        bounds = self.model.GetBounds()
        
        # Calculate model center
        center = [
            (bounds[0] + bounds[1]) / 2,
            (bounds[2] + bounds[3]) / 2,
            (bounds[4] + bounds[5]) / 2
        ]
        
        # Calculate model dimensions
        dimensions = [
            bounds[1] - bounds[0],
            bounds[3] - bounds[2],
            bounds[5] - bounds[4]
        ]
        max_dim = max(dimensions)
        
        # Set up camera
        camera = self.renderer.GetActiveCamera()
        distance = max_dim * 2.0
        
        # Use current orientation but adjust distance
        current_position = list(camera.GetPosition())
        current_focal = list(camera.GetFocalPoint())
        
        # Calculate direction vector
        direction = [
            current_position[0] - current_focal[0],
            current_position[1] - current_focal[1],
            current_position[2] - current_focal[2]
        ]
        
        # Normalize direction
        length = np.sqrt(sum(d*d for d in direction))
        if length > 0:
            direction = [d/length for d in direction]
            
            # Set new position at desired distance
            new_position = [
                center[0] + direction[0] * distance,
                center[1] + direction[1] * distance,
                center[2] + direction[2] * distance
            ]
            
            camera.SetPosition(new_position)
            camera.SetFocalPoint(center)
            
            # Reset clipping range
            self.renderer.ResetCameraClippingRange()
            self.vtk_widget.GetRenderWindow().Render()
    
    def set_view(self, view_type):
        """Set camera to a standard view"""
        if not self.model:
            return
        
        # Get model bounds
        bounds = self.model.GetBounds()
        
        # Calculate model center
        center = [
            (bounds[0] + bounds[1]) / 2,
            (bounds[2] + bounds[3]) / 2,
            (bounds[4] + bounds[5]) / 2
        ]
        
        # Calculate model dimensions
        dimensions = [
            bounds[1] - bounds[0],
            bounds[3] - bounds[2],
            bounds[5] - bounds[4]
        ]
        max_dim = max(dimensions)
        
        # Set up camera
        camera = self.renderer.GetActiveCamera()
        
        if view_type == "front":
            # Position camera in front of model (looking at X axis)
            distance = max_dim * 2.0
            camera.SetPosition(center[0] - distance, center[1], center[2])
            camera.SetFocalPoint(center)
            camera.SetViewUp(0, 0, 1)
            
        elif view_type == "side":
            # Position camera to side of model (looking at Y axis)
            distance = max_dim * 2.0
            camera.SetPosition(center[0], center[1] - distance, center[2])
            camera.SetFocalPoint(center)
            camera.SetViewUp(0, 0, 1)
        
        # Reset clipping range
        self.renderer.ResetCameraClippingRange()
        self.vtk_widget.GetRenderWindow().Render()

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = AeroStreamlinesViewer()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()