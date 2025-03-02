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
        """Load an STL file"""
        try:
            # Open file dialog
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Open STL File", "", "STL Files (*.stl)"
            )
            
            if not filename:
                return
            
            self.statusBar().showMessage(f"Loading {os.path.basename(filename)}...")
            
            # Read the STL file
            reader = vtk.vtkSTLReader()
            reader.SetFileName(filename)
            reader.Update()
            
            # Get model data
            stl_polydata = reader.GetOutput()
            
            if stl_polydata.GetNumberOfPoints() == 0:
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
            
            # Process the model (center and scale)
            self.model = self.process_model(stl_polydata)
            
            # Create mapper
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(self.model)
            
            # Create actor
            if self.model_actor:
                self.renderer.RemoveActor(self.model_actor)
            
            self.model_actor = vtk.vtkActor()
            self.model_actor.SetMapper(mapper)
            self.model_actor.GetProperty().SetColor(0.9, 0.9, 0.9)  # Light gray
            self.model_actor.GetProperty().SetSpecular(0.3)
            self.model_actor.GetProperty().SetSpecularPower(20)
            
            # Clear scene and add model
            self.renderer.RemoveAllViewProps()
            self.renderer.AddActor(self.model_actor)
            self.add_axes()
            
            # Create cell locator for the model (do this once)
            if self.locator:
                self.locator = None
            self.locator = vtk.vtkCellLocator()
            self.locator.SetDataSet(self.model)
            self.locator.BuildLocator()
            
            # Create the velocity field
            self.setup_velocity_field()
            
            # Enable animation button
            self.animation_button.setEnabled(True)
            
            # Update UI
            self.model_info.setText(f"Model: {os.path.basename(filename)}\nPoints: {num_points}\nCells: {num_cells}")
            
            # Set view to show model
            self.zoom_to_fit()
            self.set_view("side")  # Start with side view for aerodynamics
            
            self.statusBar().showMessage(f"Loaded: {os.path.basename(filename)}")
            
        except Exception as e:
            import traceback
            print(f"Error loading STL: {str(e)}")
            print(traceback.format_exc())
            self.statusBar().showMessage(f"Error: {str(e)}")
    
    def process_model(self, polydata):
        """Center and scale the model to fit well in view"""
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
        
        return transform_filter.GetOutput()
    
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
        """Update the vector field based on current airspeed"""
        if not self.vector_field or not self.model:
            return
            
        # Scale speed based on airspeed setting
        speed_factor = self.airspeed / 50.0  # 1.0 at 50% airspeed
        
        # Create velocity vectors
        vectors = vtk.vtkFloatArray()
        vectors.SetNumberOfComponents(3)  # 3D vector
        vectors.SetName("velocity")
        
        # Calculate number of points in the vector field
        dim = self.vector_field.GetDimensions()
        num_points = dim[0] * dim[1] * dim[2]
        
        # Pre-allocate the array
        vectors.SetNumberOfTuples(num_points)
        
        # For efficiency, get the bounds once
        bounds = self.model.GetBounds()
        model_center = [
            (bounds[0] + bounds[1]) / 2,
            (bounds[2] + bounds[3]) / 2,
            (bounds[4] + bounds[5]) / 2
        ]
        
        # Create a more realistic flow field with better surface interaction
        for i in range(num_points):
            # Default values - base flow in x direction
            u = 1.0 * speed_factor
            v = 0.0
            w = 0.0
            
            # Get point coordinates
            point_id = i
            point = self.vector_field.GetPoint(point_id)
            
            # Get closest point on model and distance
            closest_point = [0, 0, 0]
            cell_id = vtk.mutable(0)
            sub_id = vtk.mutable(0)
            dist2 = vtk.mutable(0.0)
            
            # Use the pre-built locator
            if self.locator:
                self.locator.FindClosestPoint(point, closest_point, cell_id, sub_id, dist2)
                distance = np.sqrt(dist2.get())
                
                # Enhanced flow behavior around the model
                if distance < 100:
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
                        
                        # Very close to surface - boundary layer effect
                        if distance < 20:
                            # Slow down significantly near surface (boundary layer)
                            boundary_factor = distance / 20.0
                            u *= boundary_factor * boundary_factor  # Quadratic slowdown
                            
                            # Calculate deflection along surface
                            # Get cell normal at the closest point
                            if cell_id.get() >= 0:
                                # Increase deflection for points very close to surface
                                deflection_strength = (1.0 - boundary_factor) * 1.0
                                
                                # Add flow components along the surface
                                # More perpendicular to the point-to-surface vector
                                perp_x = abs(direction[1]) + abs(direction[2])
                                perp_y = abs(direction[0]) + abs(direction[2])
                                perp_z = abs(direction[0]) + abs(direction[1])
                                
                                # Adjust flow to follow surface contours
                                if perp_x > 0.1:
                                    u += deflection_strength * (1.0 - abs(direction[0]))
                                if perp_y > 0.1:
                                    v += deflection_strength * (1.0 - abs(direction[1])) * np.sign(point[1] - model_center[1])
                                if perp_z > 0.1:
                                    w += deflection_strength * (1.0 - abs(direction[2])) * np.sign(point[2] - model_center[2])
                        else:
                            # Medium distance - deflection zone
                            # Use inverse square for more realistic deflection
                            deflection_factor = min(1.0, (100.0 - distance) / 80.0)
                            deflection_strength = deflection_factor * 0.7
                            
                            # Calculate flow deflection components
                            v -= direction[1] * deflection_strength
                            w -= direction[2] * deflection_strength
                            
                            # Gradual speed reduction with distance
                            u *= 0.7 + 0.3 * (distance / 100.0)
                
                # Wake effect behind the model
                if point[0] > model_center[0]:
                    # Check if point is in the potential wake zone
                    dx = point[0] - model_center[0]
                    wake_width = 100.0 + dx * 0.2  # Wake expands gradually
                    
                    # Calculate distance from wake centerline
                    dy = abs(point[1] - model_center[1])
                    dz = abs(point[2] - model_center[2])
                    
                    # Define wake boundary
                    if dy < wake_width and dz < wake_width and dx < 400:
                        # Calculate wake intensity (strongest near the model and centerline)
                        wake_intensity = (1.0 - dx/400.0) * (1.0 - (dy+dz)/(2*wake_width))
                        
                        # Reduce velocity in wake
                        u *= max(0.3, 1.0 - wake_intensity * 0.7)
                        
                        # Add some turbulence/vorticity in the wake
                        turbulence = wake_intensity * 0.2
                        v += np.sin(point[0]/20.0 + point[2]/15.0) * turbulence
                        w += np.sin(point[0]/25.0 + point[1]/15.0) * turbulence
            
            vectors.SetTuple3(i, u, v, w)
        
        # Add vectors to field
        self.vector_field.GetPointData().SetVectors(vectors)
    
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
        z_min = bounds[4] - 200
        z_max = bounds[5] + 200
        
        # Decide on number of seed points - increased for better coverage
        base_points = 10
        num_y_points = max(5, int(base_points * density_factor))
        num_z_points = max(5, int(base_points * density_factor))
        
        # Create points in a more distributed pattern around the model
        seed_points = vtk.vtkPoints()
        
        # Create a volume of seed points for more comprehensive coverage
        # Front plane (upstream)
        for i in range(num_y_points):
            y = y_min + (i / max(1, num_y_points-1)) * (y_max - y_min)
            for j in range(num_z_points):
                z = z_min + (j / max(1, num_z_points-1)) * (z_max - z_min)
                seed_points.InsertNextPoint(x_min, y, z)
        
        # Top plane
        for i in range(num_y_points):
            y = y_min + (i / max(1, num_y_points-1)) * (y_max - y_min)
            for j in range(int(num_z_points/2)):
                x = x_min + (j / max(1, int(num_z_points/2)-1)) * (x_max - x_min)
                seed_points.InsertNextPoint(x, y, z_max)
                
        # Bottom plane
        for i in range(num_y_points):
            y = y_min + (i / max(1, num_y_points-1)) * (y_max - y_min)
            for j in range(int(num_z_points/2)):
                x = x_min + (j / max(1, int(num_z_points/2)-1)) * (x_max - x_min)
                seed_points.InsertNextPoint(x, y, z_min)
                
        # Left side plane
        for i in range(int(num_y_points/2)):
            x = x_min + (i / max(1, int(num_y_points/2)-1)) * (x_max - x_min)
            for j in range(num_z_points):
                z = z_min + (j / max(1, num_z_points-1)) * (z_max - z_min)
                seed_points.InsertNextPoint(x, y_min, z)
                
        # Right side plane
        for i in range(int(num_y_points/2)):
            x = x_min + (i / max(1, int(num_y_points/2)-1)) * (x_max - x_min)
            for j in range(num_z_points):
                z = z_min + (j / max(1, num_z_points-1)) * (z_max - z_min)
                seed_points.InsertNextPoint(x, y_max, z)
        
        # Create polydata for seeds
        seed_polydata = vtk.vtkPolyData()
        seed_polydata.SetPoints(seed_points)
        
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
            if len(self.streamline_actors) < 20 and (current_time - self.last_streamline_creation > 500):
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