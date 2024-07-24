import matplotlib.pyplot as plt
import numpy as np

def read_data(filename):
    search_times = []
    compute_times = []
    update_times = []

    with open(filename, 'r') as file:
        for line in file:
            parts = line.split(': ')
            if len(parts) != 2:
                continue
            operation, time_str = parts
            try:
                time = float(time_str.split()[0])
            except ValueError:
                continue

            if operation == 'searchFrontiers':
                search_times.append(time)
            elif operation == 'computeFrontiersToVisit':
                compute_times.append(time)
            elif operation == 'updateFrontierCostMatrix':
                update_times.append(time)

    return search_times, compute_times, update_times

def filter_data(search_times, compute_times, update_times, start_index, end_index):
    # Ensure that the indices are within the range of the data lists
    search_times = search_times[start_index:end_index]
    compute_times = compute_times[start_index:end_index]
    update_times = update_times[start_index:end_index]
    
    return search_times, compute_times, update_times

def plot_combined(search_times, compute_times, update_times):
    # Create x values for plotting (sequence of calls)
    x_search = np.arange(1, len(search_times) + 1)
    x_compute = np.arange(1, len(compute_times) + 1)
    x_update = np.arange(1, len(update_times) + 1)

    plt.figure(figsize=(12, 6))
    
    # Plot all lines together
    plt.plot(x_search, search_times, 'o-', label='searchFrontiers', color='blue')
    plt.plot(x_compute, compute_times, 'o-', label='computeFrontiersToVisit', color='green')
    plt.plot(x_update, update_times, 'o-', label='updateFrontierCostMatrix', color='red')

    # Adding labels and title
    plt.xlabel('Call Number')
    plt.ylabel('Time (seconds)')
    plt.title('Execution Times for Different Functions (Combined)')
    plt.legend()
    plt.grid(True)

    # Save the combined plot
    plt.savefig('combined_plot.png')
    plt.show()

def plot_individual(search_times, compute_times, update_times):
    # Create x values for plotting (sequence of calls)
    x_search = np.arange(1, len(search_times) + 1)
    x_compute = np.arange(1, len(compute_times) + 1)
    x_update = np.arange(1, len(update_times) + 1)

    # Plot searchFrontiers
    plt.figure(figsize=(12, 6))
    plt.plot(x_search, search_times, 'o-', label='searchFrontiers', color='blue')
    plt.xlabel('Call Number')
    plt.ylabel('Time (seconds)')
    plt.title('Execution Times for searchFrontiers')
    plt.legend()
    plt.grid(True)
    plt.savefig('searchFrontiers_plot.png')
    plt.show()

    # Plot computeFrontiersToVisit
    plt.figure(figsize=(12, 6))
    plt.plot(x_compute, compute_times, 'o-', label='computeFrontiersToVisit', color='green')
    plt.xlabel('Call Number')
    plt.ylabel('Time (seconds)')
    plt.title('Execution Times for computeFrontiersToVisit')
    plt.legend()
    plt.grid(True)
    plt.savefig('computeFrontiersToVisit_plot.png')
    plt.show()

    # Plot updateFrontierCostMatrix
    plt.figure(figsize=(12, 6))
    plt.plot(x_update, update_times, 'o-', label='updateFrontierCostMatrix', color='red')
    plt.xlabel('Call Number')
    plt.ylabel('Time (seconds)')
    plt.title('Execution Times for updateFrontierCostMatrix')
    plt.legend()
    plt.grid(True)
    plt.savefig('updateFrontierCostMatrix_plot.png')
    plt.show()

if __name__ == "__main__":
    filename = 'timing_log.txt'  # Your file name here
    search_times, compute_times, update_times = read_data(filename)
    
    # Filter data for Call Numbers 300 to 500
    start_index = 299  # 0-based index (300th call is index 299)
    end_index = 500    # 0-based index (501st call is index 500)
    filtered_search_times, filtered_compute_times, filtered_update_times = filter_data(search_times, compute_times, update_times, start_index, end_index)
    
    plot_combined(filtered_search_times, filtered_compute_times, filtered_update_times)
    plot_individual(filtered_search_times, filtered_compute_times, filtered_update_times)

