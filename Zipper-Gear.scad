//Meshing Double Helix:
meshing_double_helix ();

pi=3.1415926535897932384626433832795;

//--Create Gear

module gear (
	backlash=0,
	involute_facets=0)
{
	// Pitch diameter: Diameter of pitch circle.
	pitch_diameter  =  number_of_teeth * circular_pitch / 180;
	pitch_radius = pitch_diameter/2;
	echo ("Teeth:", number_of_teeth, " Pitch radius:", pitch_radius);

	// Base Circle
	base_radius = pitch_radius*cos(pressure_angle);

	// Diametrial pitch: Number of teeth per unit length.
	pitch_diametrial = number_of_teeth / pitch_diameter;

	// Addendum: Radial distance from pitch circle to outside circle.
	addendum = 1/pitch_diametrial;

	//Outer Circle
	outer_radius = pitch_radius+addendum;

	// Dedendum: Radial distance from pitch circle to root diameter
	dedendum = addendum + clearance;

	// Root diameter: Diameter of bottom of tooth spaces.
	root_radius = pitch_radius-dedendum;
	backlash_angle = backlash / pitch_radius * 180 / pi;
	half_thick_angle = (360 / number_of_teeth - backlash_angle) / 4;

	// Variables controlling the rim.
	rim_radius = root_radius - rim_width;

	// Variables controlling the circular holes in the gear.
	circle_orbit_diameter=hub_diameter/2+rim_radius;
	circle_orbit_curcumference=pi*circle_orbit_diameter;

	// Limit the circle size to 90% of the gear face.
	circle_diameter=
		min (
			0.70*circle_orbit_curcumference/circles,
			(rim_radius-hub_diameter/2)*0.9);

	difference ()
	{
		union ()
		{
			difference ()
			{
				linear_extrude (height=rim_thickness, convexity=10, twist=twist)
				gear_shape (
					number_of_teeth,
					pitch_radius = pitch_radius,
					root_radius = root_radius,
					base_radius = base_radius,
					outer_radius = outer_radius,
					half_thick_angle = half_thick_angle,
					involute_facets=involute_facets);

				if (gear_thickness < rim_thickness)
					translate ([0,0,gear_thickness])
					cylinder (r=rim_radius,h=rim_thickness-gear_thickness+1);
			}
			if (gear_thickness > rim_thickness)
				cylinder (r=rim_radius,h=gear_thickness);
			if (hub_thickness > gear_thickness)
				translate ([0,0,gear_thickness])
				cylinder (r=hub_diameter/2,h=hub_thickness-gear_thickness);
		}
		translate ([0,0,-1])
		cylinder (
			r=bore_diameter/2,
			h=2+max(rim_thickness,hub_thickness,gear_thickness));
		if (circles>0)
		{
			for(i=[0:circles-1])	
				rotate([0,0,i*360/circles])
				translate([circle_orbit_diameter/2,0,-1])
				cylinder(r=circle_diameter/2,h=max(gear_thickness,rim_thickness)+3);
		}
	}
}

module gear_shape (
	number_of_teeth,
	pitch_radius,
	root_radius,
	base_radius,
	outer_radius,
	half_thick_angle,
	involute_facets)
{
	union()
	{
		rotate (half_thick_angle) circle ($fn=number_of_teeth*2, r=root_radius);

		for (i = [1:number_of_teeth])
		{
			rotate ([0,0,i*360/number_of_teeth])
			{
				involute_gear_tooth (
					pitch_radius = pitch_radius,
					root_radius = root_radius,
					base_radius = base_radius,
					outer_radius = outer_radius,
					half_thick_angle = half_thick_angle,
					involute_facets=involute_facets);
			}
		}
	}
}
module involute_gear_tooth (
	pitch_radius,
	root_radius,
	base_radius,
	outer_radius,
	half_thick_angle,
	involute_facets)
{
	min_radius = max (base_radius,root_radius);

	pitch_point = involute (base_radius, involute_intersect_angle (base_radius, pitch_radius));
	pitch_angle = atan2 (pitch_point[1], pitch_point[0]);
	centre_angle = pitch_angle + half_thick_angle;

	start_angle = involute_intersect_angle (base_radius, min_radius);
	stop_angle = involute_intersect_angle (base_radius, outer_radius);

	res=(involute_facets!=0)?involute_facets:($fn==0)?5:$fn/4;

	union ()
	{
		for (i=[1:res])
		assign (
			point1=involute (base_radius,start_angle+(stop_angle - start_angle)*(i-1)/res),
			point2=involute (base_radius,start_angle+(stop_angle - start_angle)*i/res))
		{
			assign (
				side1_point1=rotate_point (centre_angle, point1),
				side1_point2=rotate_point (centre_angle, point2),
				side2_point1=mirror_point (rotate_point (centre_angle, point1)),
				side2_point2=mirror_point (rotate_point (centre_angle, point2)))
			{
				polygon (
					points=[[0,0],side1_point1,side1_point2,side2_point2,side2_point1],
					paths=[[0,1,2,3,4,0]]);
			}
		}
	}
}

//--Create Zipper

module zipper(
	backlash=0,
	involute_facets=0
	)
{
	pitch_diameter  =  number_of_teeth * circular_pitch / 180;
	pitch_radius = pitch_diameter/2;

	pitch_diametrial = number_of_teeth / pitch_diameter;
	addendum = 1/pitch_diametrial;
	dedendum = addendum + clearance;
	root_radius = pitch_radius-dedendum;	

	outer_radius = pitch_radius+addendum;

	backlash_angle = backlash / pitch_radius * 180 / pi;
	half_thick_angle = (360 / number_of_teeth - backlash_angle) / 4;
	
	base_radius = pitch_radius*cos(pressure_angle);

	min_radius = max (base_radius,root_radius);
	pitch_point = involute (base_radius, involute_intersect_angle (base_radius, pitch_radius));
	pitch_angle = atan2 (pitch_point[1], pitch_point[0]);
	centre_angle = pitch_angle + half_thick_angle;
	start_angle = involute_intersect_angle (base_radius, min_radius);
	point1=involute (base_radius,start_angle);
	thickness_point=rotate_point(centre_angle,point1);
	thickness=thickness_point[0];

	tooth_thickness=thickness_point[1]*2;

	zipper_shape(
		pitch_radius=pitch_radius,
		root_radius=root_radius,
		outer_radius=outer_radius,
		half_thick_angle=half_thick_angle,
		base_radius=base_radius,
		thickness=thickness,
		involute_facets=involute_facets,
		bar_length=bar_length,
		tooth_thickness=tooth_thickness,
		number_of_teeth=number_of_teeth,
		bar_height=bar_height,
		layer_height=layer_height,
		zipper_teeth=zipper_teeth,
		twist=twist);	
}

module zipper_shape(
	pitch_radius,
	root_radius,
	outer_radius,
	half_thick_angle,
	base_radius,
	thickness,
	involute_facets,
	bar_length,
	tooth_thickness,
	number_of_teeth,
	bar_height,
	layer_height,
	zipper_teeth,
	twist)
{
	union()
	{
		translate([0,tooth_thickness/2,0])
			cube([thickness,bar_length,bar_height]);
		translate([0,bar_height,0])
			for (j=[0:layer_height:bar_height-layer_height*2])
			{
				translate([0,-j*twist*pi*pitch_radius/180/bar_height,j])
					for (i=[1:zipper_teeth])
					{
						translate([0,(i-1)*2*pi*pitch_radius/number_of_teeth,0])
						{
							involute_gear_tooth (
									pitch_radius = pitch_radius,
									root_radius = root_radius,
									base_radius = base_radius,
									outer_radius = outer_radius,
									half_thick_angle = half_thick_angle,
									involute_facets = involute_facets);
						}
					}
			}
	}
}
// Mathematical Functions
//===============

// Finds the angle of the involute about the base radius at the given distance (radius) from it's center.
//source: http://www.mathhelpforum.com/math-help/geometry/136011-circle-involute-solving-y-any-given-x.html

function involute_intersect_angle (base_radius, radius) = sqrt (pow (radius/base_radius, 2) - 1) * 180 / pi;

// Calculate the involute position for a given base radius and involute angle.

function mirror_point (coord) = 
[
	coord[0], 
	-coord[1]
];

function rotate_point (rotate, coord) =
[
	cos (rotate) * coord[0] + sin (rotate) * coord[1],
	cos (rotate) * coord[1] - sin (rotate) * coord[0]
];

function involute (base_radius, involute_angle) = 
[
	base_radius*(cos (involute_angle) + involute_angle*pi/180*sin (involute_angle)),
	base_radius*(sin (involute_angle) - involute_angle*pi/180*cos (involute_angle)),
];


// Test Cases
//===============

module meshing_double_helix ()
{
	test_double_helix_gear ();
}

module test_double_helix_gear (
	teeth=17,
	circles=8,
	zipper_teeth=17)
{
	//double helical gear
	{
		twist=200;
		height=20;
		pressure_angle=30;

		gear (number_of_teeth=teeth,
			circular_pitch=700,
			pressure_angle=pressure_angle,
			clearance = 0.2,
			gear_thickness = height/2*0.7,
			rim_thickness = height/2*1,
			rim_width = 5,
			hub_thickness = height/2*1.2,
			hub_diameter=15,
			bore_diameter=5,
			circles=circles,
			twist=twist/teeth);
		mirror([0,0,1])
			gear (number_of_teeth=teeth,
				circular_pitch=700,
				pressure_angle=pressure_angle,
				clearance = 0.2,
				gear_thickness = height/2,
				rim_thickness = height/2,
				rim_width = 5,
				hub_thickness = height/2,
				hub_diameter=15,
				bore_diameter=5,
				circles=circles,
				twist=twist/teeth);
		mirror([1,0,0])
			translate([-70,-102.5,0])
				{
				zipper (number_of_teeth=teeth,
					circular_pitch=700,
					pressure_angle=pressure_angle,
					clearance=0.2,
					twist=twist/teeth,
					bar_height=height/2,
					bar_length=250,
					layer_height=0.25,
					zipper_teeth=zipper_teeth);
				mirror([0,0,1])
				zipper (number_of_teeth=teeth,
					circular_pitch=700,
					pressure_angle=pressure_angle,
					clearance=0.2,
					twist=twist/teeth,
					bar_height=height/2,
					bar_length=250,
					layer_height=0.25,
					zipper_teeth=zipper_teeth);
				}
	}
}