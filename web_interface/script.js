function sendRequest(command, inputs) {
    const params = inputs.map(input => typeof input === 'string' ? input : input.value).join(',');
    fetch(`${command}?params=${params}`);
}

function toggleOffsets() {
    // Get all containers
    const settingsContainers = document.querySelectorAll('.offsets-container, .accel-container');
    const controlContainers = document.querySelectorAll('.walking-container, .pose-container');
    const poseGridContainer = document.querySelector('.pose-grid-container');
    
    // Get current state from button icon
    const toggleButton = document.querySelector('.toggle-button');
    const currentIcon = toggleButton.textContent;
    
    // Cycle between states
    if (currentIcon === '\u{2699}') {  // If showing gear (controls)
        // Show settings
        settingsContainers.forEach(container => container.classList.remove('hidden'));
        controlContainers.forEach(container => container.classList.add('hidden'));
        poseGridContainer.classList.add('hidden');
        toggleButton.textContent = '\u{1F3AE}';  // Game controller icon
    } else if (currentIcon === '\u{1F3AE}') {  // If showing game controller (settings)
        // Show pose grid
        settingsContainers.forEach(container => container.classList.add('hidden'));
        controlContainers.forEach(container => container.classList.add('hidden'));
        poseGridContainer.classList.remove('hidden');
        toggleButton.textContent = '\u{1F4C4}';  // Page icon
    } else {  // If showing page (pose grid)
        // Show controls
        settingsContainers.forEach(container => container.classList.add('hidden'));
        controlContainers.forEach(container => container.classList.remove('hidden'));
        poseGridContainer.classList.add('hidden');
        toggleButton.textContent = '\u{2699}';  // Gear icon
    }
}
