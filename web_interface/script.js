function sendRequest(command, inputs) {
    const params = inputs.map(input => typeof input === 'string' ? input : input.value).join(',');
    fetch(`${command}?params=${params}`);
}

function toggleOffsets() {
    // Get all containers
    const settingsContainers = document.querySelectorAll('.offsets-container, .accel-container');
    const mainContainers = document.querySelectorAll('.walking-container, .pose-container');
    
    // Check if settings are currently hidden
    const settingsHidden = document.querySelector('.offsets-container').classList.contains('hidden');
    
    // Update the button icon
    const toggleButton = document.querySelector('.toggle-button');
    toggleButton.textContent = settingsHidden ? '\u{1F3AE}' :  '\u{2699}';
    
    // Show settings and hide main containers, or vice versa
    settingsContainers.forEach(container => {
        container.classList.toggle('hidden', !settingsHidden);
    });
    
    mainContainers.forEach(container => {
        container.classList.toggle('hidden', settingsHidden);
    });
}
