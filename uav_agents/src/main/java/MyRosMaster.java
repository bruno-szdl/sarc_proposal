import embedded.mas.bridges.ros.RosMaster;
import embedded.mas.bridges.ros.DefaultRos4EmbeddedMas;
import embedded.mas.bridges.ros.ServiceParameters;
import embedded.mas.bridges.ros.ServiceParam;

import jason.asSyntax.Atom;
import jason.asSyntax.Term;
import jason.asSyntax.ListTerm;
import static jason.asSyntax.ASSyntax.parseList;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class MyRosMaster extends RosMaster {

	public MyRosMaster(Atom id, DefaultRos4EmbeddedMas microcontroller) {

		super(id, microcontroller);

	}

        /* Translate actions into ros topic publications and service requests.
           args:
             - for topic publications: 0. Topic name
                                       1. Topic type
                                       2. Topic value
                                        
             - for service requests: service arguments are the args elements. 
                                     They must be properly handled in the execEmbeddedAction function                                           
             
           obs: args are Strings. The action arguments are send to the ros as strings.
                Type conversions are handled in the "microcontroller" (DefaultRos4EmbeddedMas)       
        */
	@Override
	public boolean execEmbeddedAction(String actionName, Object[] args) {

		if(actionName.equals("goto_reference")){ 
			ServiceParam pSeq = new ServiceParam("seq", Integer.parseInt(args[1].toString())); //parameter "seq"
			ServiceParam pSecs = new ServiceParam("secs",  Integer.parseInt(args[2].toString())); //parameter "secs"
			ServiceParam pNSecs = new ServiceParam("nsecs",  Integer.parseInt(args[3].toString()));	//parameter "nsecs"	
				  
			//the parameter stamp is composed of the nested set of parameters spStamp
			ServiceParameters spStamp = new ServiceParameters();
			spStamp.add(pSecs);
			spStamp.add(pNSecs);
			ServiceParam pStamp = new ServiceParam("stamp", spStamp);  //parameter "stamp"
		 
			ServiceParam pFrame_id = new ServiceParam("frame_id",  args[4]); //parameter "frame_id"
		 
			//the parameter header is composed of the nested set of parameters spHeader
			ServiceParameters spHeader = new ServiceParameters();
			spHeader.add(pSeq);
			spHeader.add(pStamp);
			spHeader.add(pFrame_id);
			ServiceParam pHeader = new ServiceParam("header", spHeader); //parameter "header"
		 
		 
			ServiceParam pX = new ServiceParam("x", Float.parseFloat(args[5].toString())); //parameter "x"
			ServiceParam pY = new ServiceParam("y", Float.parseFloat(args[6].toString())); //parameter "y"
			ServiceParam pZ = new ServiceParam("z", Float.parseFloat(args[7].toString())); //parameter "z"
		 
		 
			//the parameter position is composed of the nested set of parameters spPosition
			ServiceParameters spPosition = new ServiceParameters();
			spPosition.add(pX);
			spPosition.add(pY);
			spPosition.add(pZ);
			ServiceParam pPosition = new ServiceParam("position", spPosition); //parameter "position"
		 
			ServiceParam pHeading = new ServiceParam("heading", Float.parseFloat(args[8].toString())); //parameter "heading"
		 
		 
			//the parameter reference is composed of the nested set of parameters spReference
			ServiceParameters spReference = new ServiceParameters();
			spReference.add(pPosition);
			spReference.add(pHeading);
			ServiceParam pReference = new ServiceParam("reference", spReference); //parameter "reference";
		 
			//final set of parameters
			ServiceParameters spFinal = new ServiceParameters();
			spFinal.add(pHeader);
			spFinal.add(pReference);
 
			serviceRequest("/uav"+args[0]+"/control_manager/reference",spFinal); 
			return true;

		}

		if(actionName.equals("generate_trajectory")){ 

			//indices dos parâmetros no vetor args:
			int indexParamSeq = 1, indexParamSecs = 2, indexParamNSecs = 3, indexParamFrame_Id = 4,
				indexParamInput_Id = 5, indexParamUseHeading = 6, indexParamFlyNow = 7, indexParamStopAtWayPoints=8,
				indexParamLoop=9, indexParamOverrideConstraints=10, indexParamOverride_max_velocity_horizontal = 11,
				indexParamOverride_max_acceleration_horizontal=12, indexParamOverride_max_jerk_horizontal=13,
				indexParamOverride_max_velocity_vertical = 14, indexParamOverride_max_acceleration_vertical=15, 
				indexParamOverride_max_jerk_vertical=16, indexParamRelax_heading=17, indexParamPoints=18, indexParamHeading=19;
				
			try{ 		     
			
				ServiceParam pSeq = new ServiceParam("seq", Integer.parseInt(args[indexParamSeq].toString())); //parameter "seq"
			    ServiceParam pSecs = new ServiceParam("secs",  Integer.parseInt(args[indexParamSecs].toString())); //parameter "secs"
			    ServiceParam pNSecs = new ServiceParam("nsecs",  Integer.parseInt(args[indexParamNSecs].toString()));	//parameter "nsecs"	
				  
			    //the parameter stamp is composed of the nested set of parameters spStamp
				ServiceParameters spStamp = new ServiceParameters();
			    spStamp.add(pSecs);
			    spStamp.add(pNSecs);
			    ServiceParam pStamp = new ServiceParam("stamp", spStamp);  //parameter "stamp"
		 
			    ServiceParam pFrame_id = new ServiceParam("frame_id",  args[indexParamFrame_Id]); //parameter "frame_id"
		 
			    //the parameter header is composed of the nested set of parameters spHeader
			    ServiceParameters spHeader = new ServiceParameters();
			    spHeader.add(pSeq);
			    spHeader.add(pStamp);
			    spHeader.add(pFrame_id);
			    ServiceParam pHeader = new ServiceParam("header", spHeader); //parameter "header"
			  
			    ListTerm list = parseList(args[indexParamPoints].toString()); 	
			    ServiceParameters[] arrayPoints = new ServiceParameters[list.size()];

			    int i=0;
			    for(Term t:list) { //percorre todos os termos da lista	
				    ServiceParam pX = new ServiceParam("x", Float.parseFloat(((ListTerm)t).get(0).toString().replaceAll("[()]", ""))); //parameter "x"
				    ServiceParam pY = new ServiceParam("y", Float.parseFloat(((ListTerm)t).get(1).toString().replaceAll("[()]", ""))); //parameter "y"
				    ServiceParam pZ = new ServiceParam("z", Float.parseFloat(((ListTerm)t).get(2).toString().replaceAll("[()]", ""))); //parameter "z"
				    //the parameter position is composed of the nested set of coordinate parameters spCoordinate
				    ServiceParameters spCoordinate = new ServiceParameters();				
				    spCoordinate.add(pX);
				    spCoordinate.add(pY);
				    spCoordinate.add(pZ);
																 
				    ServiceParam pPosition = new ServiceParam("position", spCoordinate); //parameter "position"				/
				    ServiceParameters spPosition = new ServiceParameters();
				    ServiceParam pHeading = new ServiceParam("heading", Float.parseFloat(args[indexParamHeading].toString())); //parameter "heading"
				    spPosition.add(pPosition);
				    spPosition.add(pHeading);
				    arrayPoints[i++] = spPosition;								
			    }

			    //criar um vetor com todos os parâmetros position
			    ServiceParam pPoints = new ServiceParam("points", arrayPoints); //parameter "points"			
			    ServiceParameters spPoints = new ServiceParameters();
			      
			    spPoints.add(pPoints);
			    
			    ServiceParam pInput_Id = new ServiceParam("input_id",Integer.parseInt(args[indexParamInput_Id].toString())); //param input_id
			    ServiceParam pUse_heading = new ServiceParam("use_heading",Boolean.getBoolean(args[indexParamUseHeading].toString())); //param use_heading
			    ServiceParam pFly_now = new ServiceParam("fly_now",Boolean.getBoolean(args[indexParamFlyNow].toString())); //param fly_now
			    ServiceParam pStop_at_waypoints = new ServiceParam("stop_at_waypoints",Boolean.getBoolean(args[indexParamStopAtWayPoints].toString())); //param stop_at_waypoints
			    ServiceParam pLoop = new ServiceParam("loop",Boolean.getBoolean(args[indexParamLoop].toString())); //param loop
			    ServiceParam pOverride_constraints = new ServiceParam("override_constraints",Boolean.getBoolean(args[indexParamOverrideConstraints].toString())); //param override_constraints
			    ServiceParam pOverride_max_velocity_horizontal = new ServiceParam("override_max_velocity_horizontal",Float.parseFloat(args[indexParamOverride_max_velocity_horizontal].toString())); //param override_max_acceleration_vertical
			    ServiceParam pOverride_max_acceleration_horizontal = new ServiceParam("override_max_acceleration_horizontal",Float.parseFloat(args[indexParamOverride_max_acceleration_horizontal].toString())); //param override_max_acceleration_horizontal
			    ServiceParam pOverride_max_jerk_horizontal = new ServiceParam("override_max_jerk_horizontal",Float.parseFloat(args[indexParamOverride_max_jerk_horizontal].toString())); //para  override_max_jerk_horizontal			
			    ServiceParam pOverride_max_velocity_vertical = new ServiceParam("override_max_velocity_vertical",Float.parseFloat(args[indexParamOverride_max_velocity_vertical].toString())); //param override_max_acceleration_vertical
			    ServiceParam pOverride_max_acceleration_vertical = new ServiceParam("override_max_acceleration_vertical",Float.parseFloat(args[indexParamOverride_max_acceleration_vertical].toString())); //param override_max_acceleration_vertical
			    ServiceParam pOverride_max_jerk_vertical = new ServiceParam("override_max_jerk_vertical",Float.parseFloat(args[indexParamOverride_max_jerk_vertical].toString())); //param override_max_jerk_vertical
			    ServiceParam pRelax_heading = new ServiceParam("relax_heading",Boolean.getBoolean(args[indexParamRelax_heading].toString())); //param relax_heading
			    
			    
			    //the only parameter of this service request is a set of service parameters enclosed in the variable spPath
			    ServiceParameters spPath = new ServiceParameters();
			    spPath.add(pHeader);
			    spPath.add(pInput_Id);
			    spPath.add(pUse_heading);
			    spPath.add(pFly_now);
			    spPath.add(pStop_at_waypoints);
			    spPath.add(pLoop);
			    spPath.add(pOverride_constraints);
			    spPath.add(pOverride_max_velocity_horizontal);
			    spPath.add(pOverride_max_acceleration_horizontal);
			    spPath.add(pOverride_max_jerk_horizontal);
			    spPath.add(pOverride_max_velocity_vertical);
			    spPath.add(pOverride_max_acceleration_vertical);
			    spPath.add(pOverride_max_jerk_vertical);
			    spPath.add(pRelax_heading);
			    spPath.add(pPoints);
						 
			    ServiceParam pPath = new ServiceParam("path",spPath); //param path
    
			    // "spFinal" is the set of parameters of this service request.
			    // It is composed of a single parameter "pPath" wich encloses many other parameters, as defined above
			    ServiceParameters spFinal = new ServiceParameters(); 
			    spFinal.add(pPath);
			    
			    serviceRequest("/uav"+args[0]+"/trajectory_generation/path",spFinal); 
				 
				return true;
			}

			catch (Exception e) {

		        e.printStackTrace();
				
			}
			
		}

		if(actionName.equals("goto_trajectory_start")){ //handling the action "move_turtle"

			try{		    
		    	JsonNode params = new ObjectMapper().readTree("{}"); //build a JSON with the service parameters		    
		    	((DefaultRos4EmbeddedMas) microcontroller).serviceRequest("/uav"+args[0]+"/control_manager/goto_trajectory_start", params); //send the service request		   
		   	} catch (JsonMappingException e) {
				e.printStackTrace();
		   	} catch (JsonProcessingException e) {
				e.printStackTrace();
		   	}

		}

		if(actionName.equals("start_trajectory_tracking")){ //handling the action "move_turtle"

			try{		    
				JsonNode params = new ObjectMapper().readTree("{}"); //build a JSON with the service parameters		    
				((DefaultRos4EmbeddedMas) microcontroller).serviceRequest("/uav"+args[0]+"/control_manager/start_trajectory_tracking", params); //send the service request		   
		   	} catch (JsonMappingException e) {
				e.printStackTrace();
		   	} catch (JsonProcessingException e) {
				e.printStackTrace();
		   	}

		}

		if(actionName.equals("stop_trajectory_tracking")){ //handling the action "move_turtle"

			try{		    
				JsonNode params = new ObjectMapper().readTree("{}"); //build a JSON with the service parameters		    
				((DefaultRos4EmbeddedMas) microcontroller).serviceRequest("/uav"+args[0]+"/control_manager/stop_trajectory_tracking", params); //send the service request		   
		   	} catch (JsonMappingException e) {
				e.printStackTrace();
		   	} catch (JsonProcessingException e) {
				e.printStackTrace();
		   	}

		}

		if(actionName.equals("resume_trajectory_tracking")){ //handling the action "move_turtle"

			try{		    
				JsonNode params = new ObjectMapper().readTree("{}"); //build a JSON with the service parameters		    
				((DefaultRos4EmbeddedMas) microcontroller).serviceRequest("/uav"+args[0]+"/control_manager/resume_trajectory_tracking", params); //send the service request		   
		   	} catch (JsonMappingException e) {
				e.printStackTrace();
		   	} catch (JsonProcessingException e) {
				e.printStackTrace();
		   	}

		}

		if(actionName.equals("land")){ //handling the action "move_turtle"

			try{		    
				JsonNode params = new ObjectMapper().readTree("{}"); //build a JSON with the service parameters		    
				((DefaultRos4EmbeddedMas) microcontroller).serviceRequest("/uav"+args[0]+"/uav_manager/land", params); //send the service request		   
		   	} catch (JsonMappingException e) {
				e.printStackTrace();
		   	} catch (JsonProcessingException e) {
				e.printStackTrace();
		   	}

		}

		return false;

	}
	
}
