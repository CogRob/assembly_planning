#!/bin/zsh

# for 1x1,1x2,1x3 do
#  for red, green, blue, yellow, pink do
#    create a folder megabloks1xnum_color
#    create a folder called meshes inside
#    create model.config and model.sdf using the template while replacing model_name with megabloks1xnum_color

function main_loop(){
  
  cd ../models/
  MTEMP=megabloks1x
  for num in 1 2 3
    do
      for color in '' xred xblue xgreen xpink xyellow
        do
          create_models ${MTEMP} ${num} ${color}
      done
  done

}

function create_models(){
  MNAME=$1$2$3
  mkdir -p $MNAME/meshes
  cd $MNAME
  create_config $2 $3
  create_sdf $2 $3
  cd ..
}

function update_sdf(){
  MNAME=${MTEMP}${num}${color}
  cd $MNAME
  create_sdf ${num} ${color}
  cd ..
}


function create_config(){
  echo "<?xml version=\"1.0\"?>
<model>
  <name>megabloks1x$1$2</name>
  <version>1.0</version>
  <sdf version='1.4'>model.sdf</sdf>

  <author>
   <name>Priyam Parashar</name>
   <email>priyam8parashar@gmail.com</email>
  </author>

  <description>
    A gazebo model of mega bloks 1x$1 brick in color $2" > model.config
  if [ "$1" = "1" ]; then
    echo "https://www.thingiverse.com/thing:1195909
  </description>
</model>" >> model.config
  elif [ "$1" = "2" ]; then
    echo "    https://www.thingiverse.com/thing:1195964
  </description>
</model>" >> model.config
  else
    echo "    https://www.thingiverse.com/thing:1217152
  </description>
</model>" >> model.config
  fi
}

function create_sdf(){
  echo "<?xml version='1.0'?>
<sdf version='1.4'> <!-- changed from 1.5 to be compatible with Gazebo2 -->
  <model name=\"megabloks1x$1$2\">
    <static>false</static>
    <link name=\"blok\">
      <pose>0 0 0 0 0 0</pose>
      <collision name=\"collision\">
        <geometry>
          <box>
	  <size>$((0.0313*$1)) 0.0313 0.0426</size>
          </box>
        </geometry>
	<surface>
	  <friction>
	    <ode>
	      <mu>0.4</mu>
	    </ode>
	  </friction>
	</surface>
      </collision>
      <inertial>
        <mass>$((0.003*$1))</mass>
	    <inertia>
          <ixx>0.00001</ixx>
          <iyy>0.00001</iyy>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <visual name=\"visual\">
        <pose>0 0 -0.02 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://megabloks1x$1$2/meshes/model.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>" > model.sdf
}

main_loop
