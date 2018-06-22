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
      for color in red blue green pink yellow
        do
          MNAME=${MTEMP}${num}_${color}
          cd $MNAME
          create_sdf ${num} ${color}
          cd ..
      done
  done

}

function create_models(){
  MNAME=${MTEMP}${num}_${color}
  mkdir -p $MNAME/meshes
  cd $MNAME
  create_config ${num} ${color}
  create_sdf ${num} ${color}
  cd ..
}


function create_config(){
  echo "<?xml version=\"1.0\"?>
<model>
  <name>megabloks1x$1_$2</name>
  <version>1.0</version>
  <sdf version='1.5'>model.sdf</sdf>

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
<sdf version='1.5'>
  <model name=\"megabloks1x$1_$2\">
    <static>true</static> <!--for now, will remove when we need to learn controls for this-->
    <link name=\"brick\">
      <pose>0 0 0.005  0 0 0</pose>
      <collision name=\"collision\">
        <geometry>
          <mesh>
            <uri>model://megabloks1x$1_$2/meshes/model.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name=\"visual\">
        <geometry>
          <mesh>
            <uri>model://megabloks1x$1_$2/meshes/model.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>" > model.sdf
}

main_loop