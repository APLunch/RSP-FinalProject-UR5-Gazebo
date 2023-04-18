import cv2
import numpy as np
import matplotlib.pyplot as plt
import openai
import re

# Set up the OpenAI API

openai.api_key = "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"

# Define a function to highlight specific coordinates
def highlight_coordinates(img, coordinates, color=(0, 255, 0), radius=6):
    for x, y in coordinates:
        cv2.circle(img, (x, y), radius, color, -1)

# Function to extract coordinates from the text
def extract_coordinates(text):
    coord_pattern = r'\(\d+\,\s*\d+\)'
    matches = re.findall(coord_pattern, text)
    return [tuple(map(int, coord.strip("()").split(", "))) for coord in matches]

# Load the input image
image = cv2.imread('/home/tyz/Desktop/LLM_For_Control/image_folder/cup4.png', cv2.IMREAD_GRAYSCALE)

# Detect edges using Canny Edge Detection
edges = cv2.Canny(image, 100, 200)

# Find the edge points and plot red dots
edge_points = np.argwhere(edges > 0)
image_with_dots = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
dot_spacing = 12  # Change this value to adjust the spacing between red dots

coordinates_text = ""
for i, (y, x) in enumerate(edge_points):
    if i % dot_spacing == 0:
        cv2.circle(image_with_dots, (x, y), 4, (0, 0, 255), -1)
        coordinates_text += f"({x}, {y}), "

# Add your custom text here
semantic_label = "Semantic label: cup."


# Combine the prompt components
input_prompt = f"Imagine you are a robot arm with a gripper with one degree of freedom. Now you will take a semantic label and multiple points that represent the shape of an object as inputs, and you need to output two points you want to grab, one point representing each contact point of the gripper.Please provide exactly two output coordinates representing the contact points {semantic_label} Here are the coordinates of the red dots: {coordinates_text[:-2]}.Please provide exactly two output coordinates representing the contact points, and Explain why you choose to grab that two points"

# Print the input prompt
print("Input prompt:")
print(input_prompt)


# Prepare the message
messages = [
    {
        "role": "user",
        "content": input_prompt
    }
]

# Send the message to GPT-4
response = openai.ChatCompletion.create(
    model="gpt-4-0314",
    max_tokens=100,
    temperature=0.05,
    messages=messages
)


# Extract the output coordinates from the GPT-4 response
output_coordinates_text = response.choices[0].message.content
output_coordinates = extract_coordinates(output_coordinates_text)[:2]

# Print the output results from GPT-4
print("\nOutput results from GPT-4:")
print(output_coordinates_text)

# Print the selected coordinates
print("\nSelected coordinates:")
print(output_coordinates)

# Highlight the output coordinates
highlight_coordinates(image_with_dots, output_coordinates, color=(0, 255, 0), radius=6)

# Display the final image
plt.imshow(cv2.cvtColor(image_with_dots, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()


# User Feedback Loop
while True:
    # Ask for user feedback
    feedback = input("Enter your feedback on the selected contact points or type 'stop' to exit: ")

    if feedback.lower() == "stop":
        break

    # Prepare the feedback message
    messages.append({
        "role": "user",
        "content": feedback
    })

    # Send the feedback message to GPT-4
    feedback_response = openai.ChatCompletion.create(
        model="gpt-4-0314",
        max_tokens=100,
        temperature=0.5,
        messages=messages
    )

    # Extract the output coordinates from the GPT-4 feedback response
    feedback_output_coordinates_text = feedback_response.choices[0].message.content
    feedback_output_coordinates = extract_coordinates(feedback_output_coordinates_text)[:2]

    # Print the output results from GPT-4 based on the feedback
    print("\nOutput results from GPT-4 after feedback:")
    print(feedback_output_coordinates_text)

    # Print the new selected coordinates
    print("\nNew selected coordinates:")
    print(feedback_output_coordinates)

    # Create a copy of the image with dots
    image_with_dots_feedback = image_with_dots.copy()

    # Highlight the new output coordinates
    highlight_coordinates(image_with_dots_feedback, feedback_output_coordinates, color=(255, 0, 0), radius=6)

    # Display the final image with feedback-based coordinates
    plt.imshow(cv2.cvtColor(image_with_dots_feedback, cv2.COLOR_BGR2RGB))
    plt.axis('off')
    plt.show()
