from pendulum_rewrite_controller_func2  import main
import numpy as np
import os


# folder_path = "test_A"
# A = np.linspace(0.1, 1.5, 20)

# # Create the directory
# try:
#     os.mkdir(folder_path)
#     print(f"Directory '{folder_path}' created successfully!")
# except FileExistsError:
#     print(f"Directory '{folder_path}' already exists.")
# except PermissionError:
#     print("Permission denied. Check your access rights.")

# # f = 0.5
# SH = 1.042
# VH = 0.088
# taums_H = 60

# SA = 1.4
# VA = 0.315
# taums_A = 120

# i = 0
# time = 0
# dt = 0.001

# f = 0.5 #Hz
# # for a in A:

###############test#################
# folder_path = "test_A"
# SH = 1.042
# VH = 0.2

# SA = 1.4
# VA = 0.3

# f = 1
# A = 0.1

# taums_H = 40
# taums_A = 80
# main(A, f,  SH, VH, taums_H,  SA, VA, taums_A, folder_path)
# ##############
mainF = "searchV_nextSynergyA1"
folder_path1 = mainF+"/first"
folder_path2 = mainF+"/second"
# Create the directory
try:
    os.mkdir(mainF)
    print(f"Directory '{mainF}' created successfully!")
except FileExistsError:
    print(f"Directory '{mainF}' already exists.")
except PermissionError:
    print("Permission denied. Check your access rights.")


try:
    os.mkdir(folder_path1)
    print(f"Directory '{folder_path1}' created successfully!")
except FileExistsError:
    print(f"Directory '{folder_path1}' already exists.")
except PermissionError:
    print("Permission denied. Check your access rights.")

try:
    os.mkdir(folder_path2)
    print(f"Directory '{folder_path2}' created successfully!")
except FileExistsError:
    print(f"Directory '{folder_path2}' already exists.")
except PermissionError:
    print("Permission denied. Check your access rights.")

SH = 1.2 #np.linspace(1.2, 1.5, 10) #1.2#1.042
VH = 0.5 
# # VH = np.linspace(0.09, 0.2, 10)

# SA = 1.5#1.5
# # VA = 0.3
# VA = 0.9#0.48
# f = 0.5
A = 0.6

f = 0.1
# SH = 1.042
# VH = 0.088
taums_H = 60

SA = 1.4 
VA = 0.315 
taums_A = 20



# taums_H = 50
# taums_A = 60
# for F in f:
# for sa in SA:
#     for va in VA:
main(A, f,  SH,  VH, taums_H,  SA, VA, taums_A, folder_path1, folder_path2)
#############

# folder_path = "test_f"
# f = np.linspace(0.1, 5, 50)

# # Create the directory
# try:
#     os.mkdir(folder_path)
#     print(f"Directory '{folder_path}' created successfully!")
# except FileExistsError:
#     print(f"Directory '{folder_path}' already exists.")
# except PermissionError:
#     print("Permission denied. Check your access rights.")

# A = 0.2
# S = 1.5
# V = 0.25
# taums = 60

# for F in f:
#         main(A, F, S, V, taums, "test_f")


# ##############

# folder_path = "test_S_175_V_var"
# S = 1.75#np.linspace(1.1, 2.0, 20)
# V = np.linspace(0.12, 0.7, 50)

# try:
#     os.mkdir(folder_path)
#     print(f"Directory '{folder_path}' created successfully!")
# except FileExistsError:
#     print(f"Directory '{folder_path}' already exists.")
# except PermissionError:
#     print("Permission denied. Check your access rights.")

# A = 0.3
# f = 0.5
# taums = 60

# # for s in S:
# #     for v in V:
# #         main(A, f, s, v, taums, "test_S_V")
# for v in V:
#     main(A, f, S, v, taums, "test_S_175_V_var")
##############

# folder_path = "test_tau"


# try:
#     os.mkdir(folder_path)
#     print(f"Directory '{folder_path}' created successfully!")
# except FileExistsError:
#     print(f"Directory '{folder_path}' already exists.")
# except PermissionError:
#     print("Permission denied. Check your access rights.")

# A = 0.2
# S = 1.5
# V = 0.25
# f=0.5
# taums = np.array(range(5, 90))

# for t in taums:
    
#     main(A, f, S, V, t, "test_tau")