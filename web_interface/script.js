function sendRequest(command, inputs) {
    const params = Array.from(inputs).map(input => input.value).join(',');
    fetch(`${command}?params=${params}`);
} 