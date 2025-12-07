
        function startSystem() {
          fetch('http://localhost:5000/start', { method: 'POST' })
            .then(response => response.json())
            .then(data => console.log('Start response:', data))
            .catch(err => console.error('Start error:', err));
        }
      
        function stopSystem() {
          fetch('http://localhost:5000/stop', { method: 'POST' })
            .then(response => response.json())
            .then(data => console.log('Stop response:', data))
            .catch(err => console.error('Stop error:', err));
        }
     