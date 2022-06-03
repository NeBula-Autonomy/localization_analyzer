"""
Description:  
    - Simple analysis of base rerun data. Mostly loop closure evaluations. 
Author: 
    - Yun Chang
"""

import sys
import rospkg
import shutil

sys.path.insert(1, rospkg.RosPack().get_path('localization_analyzer') + "/utilities")
from utilities import *

def aggregate_odom(data_dir, gt_dir, robot_name, run_number):
    bag_topics = {}
    # ground truth bag and topic 
    bag_topics['ground_truth'] = [gt_dir + "/odometry.bag", "/" + robot_name + "/lo_frontend/odometry"]
    bag_topics['LO'] = [data_dir + "/" + robot_name + "_locus/odometry.bag", "/" + robot_name + "/lo_frontend/odometry"]
    bag_topics['LAMP'] = [data_dir + "/lamp_" + run_number + "/odometry.bag", "/" + robot_name + "/lamp/odometry"]

    with rosbag.Bag("aggregated_odometries.bag", "w") as outbag: 
        for key in bag_topics.keys():
            print("Aggregating " + key + " ...")
            bag = rosbag.Bag(bag_topics[key][0])
            if key != "ground_truth":
                bag_topics[key][1] = bag.get_type_and_topic_info()[1].keys()[0]
            print("Topic for " + key + " odometry is " + bag_topics[key][1] + "\n")
            for topic, msg, t in bag.read_messages(topics=[bag_topics[key][1]]):
                outbag.write(key, msg, t)
    outbag.close()

def creatOutputPdf(results_dir):

    # Convert images desired into pdf 
    ape_rot = results_dir + "/plots/APERotvsDistance.png"
    imageToPDF(ape_rot)
    ape_trans = results_dir + "/plots/APETransvsDistance.png"
    imageToPDF(ape_trans)

    # Create pdf and start adding pages
    from PyPDF2 import PdfFileReader, PdfFileWriter
    pdf_writer = PdfFileWriter()

    # Add trajectory
    input_pdf = PdfFileReader(results_dir + "/plots/traj_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(0))

    # Add colored error plots
    input_pdf = PdfFileReader(results_dir + "/plots/LAMP_ape_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(1))
    
    # Add error vs distance
    input_pdf = PdfFileReader(results_dir + "/plots/APETransvsDistance.pdf")
    pdf_writer.addPage(input_pdf.getPage(0))
    input_pdf = PdfFileReader(results_dir + "/plots/APERotvsDistance.pdf")
    pdf_writer.addPage(input_pdf.getPage(0))

    # Add box plot
    input_pdf = PdfFileReader(results_dir + "/plots/combined_ape_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(3))

    # More Stats on overall error
    input_pdf = PdfFileReader(results_dir + "/plots/combined_ape_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(1))
    pdf_writer.addPage(input_pdf.getPage(4))

    # X, Y, Z error
    input_pdf = PdfFileReader(results_dir + "/plots/traj_plots.pdf")
    pdf_writer.addPage(input_pdf.getPage(1))

    # Write to file 
    with open(results_dir + "/main_results.pdf", 'wb') as outfile:
        pdf_writer.write(outfile)


def main():

    global methods 
        
    if len(sys.argv)<4:
        print("Example Usage: python base_analyzer.py data_dir base_name run_number")
        sys.exit(1)  

    data_dir = sys.argv[1]
    base_name = sys.argv[2]
    run_number = sys.argv[3]
    params = {}

    settings_filename = rospkg.RosPack().get_path('localization_analyzer') + "/config/base_analyzer_settings.yaml"

    with open(settings_filename) as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
        for k in params:
            print(k + " : " + str(params[k])) 
    print("\n")

    print("\n#######################")
    print("Starting Base Analyzer")
    print("#######################\n")

    b_clear_results_dir = params["b_clear_results_dir"]
    b_compute_ape = params["b_compute_ape"]
    b_align_origin = params["b_align_origin"]

    b_plot_combined_ape = params["b_plot_combined_ape"]
    b_show_ape_plots = params["b_show_ape_plots"]
    b_save_individual_ape_plots = params["b_save_individual_ape_plots"]
    b_save_combined_ape_plots = params["b_save_combined_ape_plots"]
    b_compute_rpe = params["b_compute_rpe"]
    b_plot_rpe = params["b_plot_rpe"]
    checkpoints  = params["checkpoints"]
    b_compute_ape_over_distances = params["b_compute_ape_over_distances"]   
    b_plot_ape_over_distances = params["b_plot_ape_over_distances"]
    b_save_results = params["b_save_results"]

    # Distance metrics
    b_compute_distance_based_metrics = params["b_compute_distance_based_metrics"]
    b_show_distance_plots = params["b_show_distance_plots"]

    b_save_traj_plots = ["b_save_traj_plots"]


    robots = []
    gt_files = []
    for i in range(len(params["robots"])):
        robots.append(params["robots"][i])
        gt_files.append(data_dir + "/ground_truth/" + params["robots"][i])

    results_dir = data_dir + "/results_" + run_number + "/"
    if not os.path.exists(results_dir):
        os.mkdir(results_dir)
    os.chdir(results_dir)

    # analyze loop closures 
    pose_graph_rosbag = data_dir + "/" + "lamp" + "_" + run_number + "/pose_graph.bag"
    pose_graph_topic = "/" + base_name + "/lamp/pose_graph"
    loop_closure_rosbag = data_dir + "/" + "lamp" + "_" + run_number + "/loop_closures.bag"
    loop_closure_topic = "/" + base_name + "/lamp/laser_loop_closures"
    get_loop_closures_stats(
        pose_graph_rosbag, pose_graph_topic, loop_closure_rosbag, loop_closure_topic, results_dir)

    # If we do not have ground truth
    if not b_compute_ape: 
        return

    methods = ["LO", "LAMP"]
    all_methods_string = " "
    for method in methods: 
        all_methods_string = all_methods_string + method + " "

    # Convert from pose graph to odom for each robot 
    for i in range(len(robots)):
        robot_name = robots[i]
        gt_file = gt_files[i]

        results_dir = data_dir + "/results_" + run_number + "/" + robot_name + "/"
        if not os.path.exists(results_dir):
            os.mkdir(results_dir)
        else:
            # Clean up
            if b_clear_results_dir:
                if os.path.exists(results_dir + "combined_ape.csv"):
                    os.remove(results_dir + "combined_ape.csv")
                if os.path.exists(results_dir + "trans_ape.csv"):
                    os.remove(results_dir + "trans_ape.csv")
                if os.path.exists(results_dir + "rot_ape.csv"):
                    os.remove(results_dir + "rot_ape.csv")
                if os.path.exists(results_dir + "main_results.csv"):
                    os.remove(results_dir + "main_results.csv")

        os.chdir(results_dir)

        # Make and clean plot directory
        plot_dir = results_dir + "plots"
        if not os.path.exists(plot_dir):
            os.mkdir(plot_dir)
        else:
            shutil.rmtree(plot_dir)
            os.mkdir(plot_dir)

        pose_graph_to_odometry(data_dir + "/" + "lamp" + "_" + run_number + "/", robot_name)
        aggregate_odom(data_dir, gt_file, robot_name, run_number)

        #-----------------------------------------------------------------------------------------------------------
        ### Plot trajectories ###
        #-----------------------------------------------------------------------------------------------------------
            
        bash_command = "evo_config set plot_fontfamily arial plot_fontscale 1.5 plot_seaborn_style whitegrid plot_figsize 14 10"
        os.system(bash_command)
        evo_options_string = make_evo_options_string("traj", "", b_align_origin, b_show_ape_plots, b_save_traj_plots)
        bash_command = "evo_traj bag aggregated_odometries.bag ground_truth" + all_methods_string + "--ref ground_truth " + evo_options_string
        os.system(bash_command)

        #-----------------------------------------------------------------------------------------------------------
        ### APE Processing with EVO ###
        #-----------------------------------------------------------------------------------------------------------
        if b_compute_ape:
            # Create directory
            ape_dir = results_dir + "/ape_results"
            if not os.path.exists(ape_dir):
                os.mkdir(ape_dir)
            else:
                shutil.rmtree(ape_dir)
                os.mkdir(ape_dir)


            # Get APE for each method
            for method in methods:
                # Make options string from params
                evo_options_string = make_evo_options_string("single", method, b_align_origin, b_show_ape_plots, b_save_individual_ape_plots)

                # Run bash command to compute APE for each method
                bash_command = "evo_ape bag aggregated_odometries.bag ground_truth " + method + " " + evo_options_string + " --save_results " + ape_dir + "/" + method + ".zip"
                os.system(bash_command)
            if b_plot_combined_ape:
                # Make options string from params
                evo_options_string = make_evo_options_string("combined", "", False, b_show_ape_plots, b_save_combined_ape_plots)

                # Run bash command to show combined results
                bash_command = "evo_res ape_results/*.zip --use_filenames" + " " + evo_options_string
                os.system(bash_command)
            for method in methods:
                # Run for translation only
                evo_options_string = make_evo_options_string("trans", method, b_align_origin, b_show_ape_plots, b_save_individual_ape_plots)
                bash_command = "evo_ape bag aggregated_odometries.bag ground_truth " + method + " --pose_relation trans_part" + evo_options_string + " --save_results " + ape_dir + "/" + method + "_trans.zip"
                os.system(bash_command)
                # Run for rotation only
                evo_options_string = make_evo_options_string("rot", method, b_align_origin, b_show_ape_plots, b_save_individual_ape_plots)
                bash_command = "evo_ape bag aggregated_odometries.bag ground_truth " + method + " --pose_relation angle_deg" + evo_options_string + " --save_results " + ape_dir + "/" + method + "_rot.zip"
                os.system(bash_command)
            # Run bash command to show combined results
            evo_options_string = make_evo_options_string("combined_trans", "", False, False, b_save_combined_ape_plots)
            bash_command = "evo_res ape_results/*trans.zip --use_filenames" + " " + evo_options_string
            os.system(bash_command)
            evo_options_string = make_evo_options_string("combined_rot", "", False, False, b_save_combined_ape_plots)
            bash_command = "evo_res ape_results/*rot.zip --use_filenames" + " " + evo_options_string
            os.system(bash_command)

        #-----------------------------------------------------------------------------------------------------------
        ### Distance based error metrics ###
        #-----------------------------------------------------------------------------------------------------------
        if b_compute_distance_based_metrics:
            # Extract zip contents
            dest = results_dir + "ape_results"
            for el in os.listdir(dest): 
                if os.path.isfile(dest + "/" + el) and el.split(".")[1] == "zip": 
                    with zipfile.ZipFile(dest + "/" + el, 'r') as zip_ref:
                        zip_ref.extractall(dest + "/" + el.split(".")[0])
            
            # Compute the distance metrics
            err_perc = compute_distance_based_error_metrics(methods, results_dir, b_show_distance_plots, False)


        #-----------------------------------------------------------------------------------------------------------
        ### APE At checkpoints with EVO ###
        #-----------------------------------------------------------------------------------------------------------
        if b_compute_ape_over_distances: 
            segment_aggregated_odometries(methods, checkpoints)
            for checkpoint in checkpoints:
                bash_command = "bash $(rospack find localization_analyzer)/utilities/ape_from_aggregated_odometries_parser.bash " + \
                               "aggregated_odometries_" + str(checkpoint) + ".bag" + " " + \
                               "ape_results_" + str(checkpoint) + " " + all_methods_string 
                os.system(bash_command)
                dest = os.getcwd() + "/ape_results_" + str(checkpoint)
                for el in os.listdir(dest): 
                    if os.path.isfile(dest + "/" + el) and el.split(".")[1] == "zip": 
                        with zipfile.ZipFile(dest + "/" + el, 'r') as zip_ref:
                            zip_ref.extractall(dest + "/" + el.split(".")[0])
            if b_plot_ape_over_distances:
                boxplot_from_npz(methods, checkpoints, results_dir, b_show_ape_plots, False)

        #-----------------------------------------------------------------------------------------------------------
        ### RPE Processing with EVO ###
        #-----------------------------------------------------------------------------------------------------------
        if b_compute_rpe: 
            # Create directory
            rpe_dir = results_dir + "/rpe_results"
            if not os.path.exists(rpe_dir):
                os.mkdir(rpe_dir)
            else:
                shutil.rmtree(rpe_dir)
                os.mkdir(rpe_dir)

            # Get RPE for each method
            for method in methods:
                # Make options string from params
                # evo_options_string = make_evo_options_string("single", method, b_align_origin, b_show_ape_plots, b_save_individual_ape_plots)

                # Run bash command to compute APE for each method
                bash_command = "evo_rpe bag aggregated_odometries.bag ground_truth " + method + " --delta 1 --delta_unit m --save_results " + rpe_dir + "/" + method + ".zip"
                os.system(bash_command)
            if b_plot_rpe:
                # Make options string from params
                # evo_options_string = make_evo_options_string("combined", "", b_align_origin, b_show_rpe_plots, b_plot_rpe)

                # Run bash command to show combined results
                bash_command = "evo_res rpe_results/*.zip --merge --use_filenames" #+ " " + evo_options_string
                os.system(bash_command)


            # bash_command = "bash $(rospack find localization_analyzer)/utilities/rpe_from_aggregated_odometries.bash " + all_methods_string 
            # os.system(bash_command)
            # if b_plot_rpe:
            #     bash_command = "bash $(rospack find localization_analyzer)/utilities/plot_rpe_results.bash " + all_methods_string 
            #     os.system(bash_command)

        #-----------------------------------------------------------------------------------------------------------
        ### Export main stats to CSV ###
        #-----------------------------------------------------------------------------------------------------------
        if b_save_results:
            creatOutputPdf(results_dir)

        # Remove aggregated_odometries
        os.remove(results_dir + "aggregated_odometries.bag")




if __name__=="__main__":
    main()